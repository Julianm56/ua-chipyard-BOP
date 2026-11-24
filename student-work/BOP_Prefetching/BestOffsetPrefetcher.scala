// Best-Offset Prefetcher (BOP)
//
// Learns from L1 D$ misses and prefetches into the L2 using TileLink Hints.
// All addresses in this file are block numbers.
//
// On each L1 miss X:
//   - Learn: test one offset d. If X - d is in the RR table, add 1 to d's score.
//   - Prefetch: if prefetching is on, request X + D.
//
// When a prefetch arrives, its trigger X is written to the RR table. So finding
// X - d there means offset d would have been early enough.
//
// A phase ends when a score reaches scoreMax or after roundMax rounds. Then
// D = best offset, and prefetching turns off if the best score is too low.

package bop

import chisel3._
import chisel3.util._
import chisel3.experimental.IntParam

// Parameters
case class BestOffsetParams(
  // Default: 1..63 with only 2, 3, 5 as factors.
  offsets: Seq[Int] = BestOffsetParams.smoothOffsets(maxOffset = 63),
  scoreMax: Int = 31,              // end when score reached
  roundMax: Int = 100,             // end after this many rounds
  badScore: Int = 1,               // best score
  rrEntries: Int = 256,            // RR table size
  rrTagBits: Int = 12,             // RR tag width
  pageBytes: Option[Int] = Some(4096),
  learnSamePage: Boolean = true,
  initialOffset: Int = 1,          // D before the first phase ends
  initiallyOn: Boolean = true,     // prefetching on before the first phase ends
  queueDepth: Int = 4,             // request queue, new requests dropped when full
  prefetchWrites: Boolean = false, // send PREFETCH_WRITE for store misses
  simMonitor: Boolean = true       // print a [BOP] line per phase in simulation
) {
  require(offsets.nonEmpty, "BOP needs at least one candidate offset")
  require(!offsets.contains(0), "offset 0 is not a prefetch")
  require(offsets.distinct.size == offsets.size, "duplicate offsets in the BOP offset list")
  require(isPow2(rrEntries), "rrEntries must be a power of two")
  require(scoreMax >= 1 && roundMax >= 1 && badScore >= 0 && badScore < scoreMax)
  require(queueDepth >= 1)
  require(initialOffset != 0 && initialOffset.abs <= offsets.map(_.abs).max,
    "initialOffset must be nonzero and no larger than the largest offset in the list")
}

object BestOffsetParams {
  // True if n has no prime factors other than 2, 3, 5
  def isSmooth(n: Int): Boolean = {
    var m = n
    for (p <- Seq(2, 3, 5)) { while (m % p == 0) { m = m / p } }
    m == 1
  }

  // Offsets 1..maxOffset with only 2, 3, 5 as factors
  def smoothOffsets(maxOffset: Int, negative: Boolean = false): Seq[Int] = {
    val pos = (1 to maxOffset).filter(isSmooth)
    if (negative) { pos.flatMap(o => Seq(o, -o)) } else { pos }
  }
}

// Interface
class BOPSnoop(val blockBits: Int) extends Bundle {
  val block = UInt(blockBits.W)   // missed block X
  val write = Bool()              // miss was a store
}

class BOPRequest(val blockBits: Int) extends Bundle {
  val block = UInt(blockBits.W)   // block to prefetch, X + D
  val base = UInt(blockBits.W)    // trigger X, returned when the prefetch arrives
  val write = Bool()              // use PREFETCH_WRITE
}

class BOPFill(val blockBits: Int) extends Bundle {
  val base = UInt(blockBits.W)    // prefetch: its trigger X; demand miss: the block
  val prefetch = Bool()           // true = prefetch arrived, false = demand miss filled
}

class BOPIO(val blockBits: Int) extends Bundle {
  val snoop = Flipped(Valid(new BOPSnoop(blockBits)))
  val fill = Flipped(Valid(new BOPFill(blockBits)))
  val request = Decoupled(new BOPRequest(blockBits))
}

// Prefetcher
class BestOffsetPrefetcher(
  val params: BestOffsetParams,
  val blockBits: Int,   // bits in a block number
  val blockBytes: Int,  // cache line size
  val monitorId: Int = 0
) extends Module {

  val io = IO(new BOPIO(blockBits))

  // Sizes
  val nOffsets = params.offsets.size
  val maxAbsOffset = params.offsets.map(_.abs).max
  val offW = log2Ceil(maxAbsOffset + 1) + 1       
  val scoreW = log2Ceil(params.scoreMax + 1)
  val roundW = log2Ceil(params.roundMax + 1)
  val idxW = log2Ceil(nOffsets max 2)
  val rrIdxBits = log2Ceil(params.rrEntries)
  val hashW = Seq(blockBits + 2, 2 * rrIdxBits, rrIdxBits + params.rrTagBits).max
  val pageBlockBits: Option[Int] = params.pageBytes.map { pb =>
    require(isPow2(pb) && pb >= blockBytes, s"pageBytes ($pb) must be a power of two >= blockBytes ($blockBytes)")
    log2Ceil(pb / blockBytes)
  }
  pageBlockBits.foreach { pbb =>
    val tooBig = params.offsets.filter(_.abs >= (1 << pbb))
    require(tooBig.isEmpty,
      s"BOP offsets ${tooBig.mkString(",")} can never be issued inside a ${params.pageBytes.get}-byte page; " +
      "drop them or set pageBytes = None")
  }

  require(blockBits > 0)
  require(isPow2(blockBytes))
  require(offW <= 16 && params.scoreMax < 256 && params.roundMax < 65536, "monitor port widths")

  // Register the inputs (adds one cycle)
  val s1Valid = RegNext(io.snoop.valid, false.B)
  val s1 = RegEnable(io.snoop.bits, io.snoop.valid)
  val f1Valid = RegNext(io.fill.valid, false.B)
  val f1 = RegEnable(io.fill.bits, io.fill.valid)

  // State
  val offsetVec = VecInit(params.offsets.map(o => o.S(offW.W)))

  val prefetchOn = RegInit(params.initiallyOn.B)
  val bestOffset = RegInit(params.initialOffset.S(offW.W))   // D

  val scores = RegInit(VecInit(Seq.fill(nOffsets)(0.U(scoreW.W))))
  val testIdx = RegInit(0.U(idxW.W))        // offset being tested
  val round = RegInit(0.U(roundW.W))        // rounds done this phase
  val phaseBestScore = RegInit(0.U(scoreW.W))
  val phaseBestIdx = RegInit(0.U(idxW.W))

  val rrTag = Reg(Vec(params.rrEntries, UInt(params.rrTagBits.W)))
  val rrValid = RegInit(VecInit(Seq.fill(params.rrEntries)(false.B)))

  // RR index and tag
  def rrIndex(b: UInt): UInt = {
    val x = b.pad(hashW)
    x(rrIdxBits - 1, 0) ^ x(2 * rrIdxBits - 1, rrIdxBits)
  }
  def rrTagOf(b: UInt): UInt = {
    val x = b.pad(hashW)
    x(rrIdxBits + params.rrTagBits - 1, rrIdxBits)
  }

  // RR write: prefetch arrivals while on, demand fills while off
  val rrWrite = f1Valid && Mux(f1.prefetch, prefetchOn, !prefetchOn)
  when (rrWrite) {
    rrTag(rrIndex(f1.base)) := rrTagOf(f1.base)
    rrValid(rrIndex(f1.base)) := true.B
  }

  // Learning
  val x = s1.block
  val testOffset = offsetVec(testIdx)
  val lookupS = x.zext -& testOffset                // X - d
  val lookup = lookupS.asUInt
  val lookupIdx = rrIndex(lookup)
  val lookupSamePage = pageBlockBits match {
    case Some(pbb) if params.learnSamePage => (lookupS >> pbb) === (x.zext >> pbb)
    case _                                 => true.B
  }
  val rrHit = lookupSamePage && rrValid(lookupIdx) && rrTag(lookupIdx) === rrTagOf(lookup)

  val curScore = scores(testIdx)
  val newScore = Mux(rrHit && curScore =/= params.scoreMax.U, curScore + 1.U, curScore)
  val improves = rrHit && newScore > phaseBestScore
  val nextBestScore = Mux(improves, newScore, phaseBestScore)
  val nextBestIdx = Mux(improves, testIdx, phaseBestIdx)
  val lastInRound = testIdx === (nOffsets - 1).U
  val phaseEnd = s1Valid &&
    (nextBestScore >= params.scoreMax.U || (lastInRound && round === (params.roundMax - 1).U))

  when (s1Valid) {
    scores(testIdx) := newScore
    phaseBestScore := nextBestScore
    phaseBestIdx := nextBestIdx
    testIdx := Mux(lastInRound, 0.U, testIdx + 1.U)
    round := Mux(lastInRound, round + 1.U, round)
  }

  when (phaseEnd) {
    bestOffset := offsetVec(nextBestIdx)
    prefetchOn := nextBestScore > params.badScore.U
    // Reset for the next phase
    scores.foreach(_ := 0.U)
    phaseBestScore := 0.U
    phaseBestIdx := 0.U
    testIdx := 0.U
    round := 0.U
  }

  // Prefetch: X + D
  val xS = x.zext
  val yS = xS +& bestOffset
  val yInRange = yS >= 0.S && (yS.asUInt >> blockBits) === 0.U
  val samePage = pageBlockBits match {
    case Some(pbb) => (yS >> pbb) === (xS >> pbb)
    case None      => true.B
  }
  val yU = yS.asUInt
  val y = yU(blockBits - 1, 0)

  // Skip a request identical to the previous one
  val lastIssued = RegInit(0.U(blockBits.W))
  val lastIssuedValid = RegInit(false.B)
  val duplicate = lastIssuedValid && lastIssued === y

  val queue = Module(new Queue(new BOPRequest(blockBits), params.queueDepth))
  val wantIssue = s1Valid && prefetchOn && yInRange && samePage && !duplicate
  queue.io.enq.valid := wantIssue
  queue.io.enq.bits.block := y
  queue.io.enq.bits.base := x
  queue.io.enq.bits.write := (if (params.prefetchWrites) s1.write else false.B)
  when (queue.io.enq.fire) {
    lastIssued := y
    lastIssuedValid := true.B
  }
  io.request <> queue.io.deq

  // Counters and simulation printout
  if (params.simMonitor) {
    val cycle = RegInit(0.U(64.W))
    cycle := cycle + 1.U
    val nPhases = RegInit(0.U(32.W))
    val nTriggers = RegInit(0.U(32.W))
    val nIssued = RegInit(0.U(32.W))
    val nDropped = RegInit(0.U(32.W))    // queue was full
    val nPageCross = RegInit(0.U(32.W))  // X + D was in another page
    val nPfFills = RegInit(0.U(32.W))
    when (s1Valid) { nTriggers := nTriggers + 1.U }
    when (queue.io.enq.fire) { nIssued := nIssued + 1.U }
    when (wantIssue && !queue.io.enq.ready) { nDropped := nDropped + 1.U }
    when (s1Valid && prefetchOn && !(yInRange && samePage)) { nPageCross := nPageCross + 1.U }
    when (f1Valid && f1.prefetch) { nPfFills := nPfFills + 1.U }
    when (phaseEnd) { nPhases := nPhases + 1.U }

    val mon = Module(new BOPMonitor(monitorId))
    mon.io.clock := clock
    mon.io.reset := reset.asBool
    mon.io.valid := phaseEnd
    mon.io.cycle := cycle
    mon.io.phase := nPhases + 1.U
    mon.io.offset := offsetVec(nextBestIdx).pad(16).asUInt
    mon.io.score := nextBestScore.pad(8)
    mon.io.rounds := (round + lastInRound).pad(16)
    mon.io.on := nextBestScore > params.badScore.U
    mon.io.triggers := nTriggers + 1.U
    mon.io.issued := nIssued
    mon.io.dropped := nDropped
    mon.io.pageCross := nPageCross
    mon.io.pfFills := nPfFills
  }
}

// Simulation printout
// Prints one line per phase.
class BOPMonitor(id: Int) extends BlackBox(Map("ID" -> IntParam(id))) with HasBlackBoxInline {
  val io = IO(new Bundle {
    val clock = Input(Clock())
    val reset = Input(Bool())
    val valid = Input(Bool())
    val cycle = Input(UInt(64.W))
    val phase = Input(UInt(32.W))
    val offset = Input(UInt(16.W))   // signed
    val score = Input(UInt(8.W))
    val rounds = Input(UInt(16.W))
    val on = Input(Bool())
    val triggers = Input(UInt(32.W))
    val issued = Input(UInt(32.W))
    val dropped = Input(UInt(32.W))
    val pageCross = Input(UInt(32.W))
    val pfFills = Input(UInt(32.W))
  })

  setInline("BOPMonitor.v",
    """module BOPMonitor #(parameter ID = 0) (
      |  input         clock,
      |  input         reset,
      |  input         valid,
      |  input  [63:0] cycle,
      |  input  [31:0] phase,
      |  input  [15:0] offset,
      |  input  [7:0]  score,
      |  input  [15:0] rounds,
      |  input         on,
      |  input  [31:0] triggers,
      |  input  [31:0] issued,
      |  input  [31:0] dropped,
      |  input  [31:0] pageCross,
      |  input  [31:0] pfFills
      |);
      |`ifndef SYNTHESIS
      |  always @(posedge clock) begin
      |    if (!reset && valid) begin
      |      if (on)
      |        $display("[BOP%0d] cycle=%0d phase=%0d D=%0d score=%0d rounds=%0d prefetch=ON triggers=%0d issued=%0d dropped=%0d pagecross=%0d pf_fills=%0d",
      |                 ID, cycle, phase, $signed(offset), score, rounds, triggers, issued, dropped, pageCross, pfFills);
      |      else
      |        $display("[BOP%0d] cycle=%0d phase=%0d D=%0d score=%0d rounds=%0d prefetch=OFF triggers=%0d issued=%0d dropped=%0d pagecross=%0d pf_fills=%0d",
      |                 ID, cycle, phase, $signed(offset), score, rounds, triggers, issued, dropped, pageCross, pfFills);
      |    end
      |  end
      |`endif
      |endmodule
      |""".stripMargin)
}
