//   - Prefetch arrived: the L2 sends HintAck once the line is in L2.
//     We return the trigger X that started that prefetch.
//   - Demand miss filled: the first beat of GrantData.

package bop

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.{Config, Field, Parameters}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.util._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.subsystem._

case class BOPTLPrefetcherParams(
  prefetchIds: Int = 4,  // max prefetches in flight
  // Which clients get a BOP
  prefetcher: String => Option[BestOffsetParams] = _ => None
)

case object BOPTLPrefetcherKey extends Field[BOPTLPrefetcherParams](BOPTLPrefetcherParams())

class BOPTLPrefetcher(implicit p: Parameters) extends LazyModule {
  val params = p(BOPTLPrefetcherKey)

  def mapInputIds(masters: Seq[TLMasterParameters]) =
    TLXbar.assignRanges(masters.map(_.sourceId.size + params.prefetchIds))

  val node = TLAdapterNode(
    clientFn = { cp =>
      cp.v1copy(clients = (mapInputIds(cp.clients) zip cp.clients).map { case (range, c) => c.v1copy(
        sourceId = range
      )})
    },
    managerFn = { mp => mp }
  )

  // Low bits of an id (0 when size is 1)
  def trim(id: UInt, size: Int): UInt = if (size <= 1) 0.U else id(log2Ceil(size)-1, 0)

  lazy val module = new LazyModuleImp(this) {
    (node.in zip node.out) foreach { case ((in, edgeIn), (out, edgeOut)) =>
      val nClients = edgeOut.master.clients.size
      val outIdMap = edgeOut.master.clients.map(_.sourceId)
      val inIdMap = edgeIn.master.clients.map(_.sourceId)

      val blockBytes = p(CacheBlockBytes)
      val lgBlock = log2Ceil(blockBytes)
      val addrBits = edgeOut.bundle.addressBits
      val blockBits = addrBits - lgBlock
      def toBlock(addr: UInt): UInt = {
        val shifted = addr >> lgBlock
        shifted(blockBits - 1, 0)
      }
      def toAddr(block: UInt): UInt = Cat(block, 0.U(lgBlock.W))

      def inIdAdjuster(source: UInt) = Mux1H((inIdMap zip outIdMap).map { case (i,o) =>
        i.contains(source) -> (o.start.U | (source - i.start.U))
      })
      def outIdAdjuster(source: UInt) = Mux1H((inIdMap zip outIdMap).map { case (i,o) =>
        o.contains(source) -> (trim(source - o.start.U, i.size) + i.start.U)
      })
      def outIdToPrefetchId(source: UInt) = Mux1H((inIdMap zip outIdMap).map { case (i,o) =>
        o.contains(source) -> trim(source - (o.start + i.size).U, params.prefetchIds)
      })
      def prefetchIdToOutId(source: UInt, client: UInt) = Mux1H((inIdMap zip outIdMap).zipWithIndex.map { case ((i,o),id) =>
        (id.U === client) -> ((o.start + i.size).U +& source)
      })
      def inIdToClientId(source: UInt) = Mux1H(inIdMap.zipWithIndex.map { case (i,id) =>
        i.contains(source) -> id.U
      })
      def outIdToClientId(source: UInt) = Mux1H(outIdMap.zipWithIndex.map { case (o,id) =>
        o.contains(source) -> id.U
      })

      // Watch L1 requests to cacheable memory
      val snoop = Wire(Valid(new BOPSnoop(blockBits)))
      snoop.valid := in.a.fire && edgeIn.manager.supportsAcquireBFast(in.a.bits.address, lgBlock.U)
      snoop.bits.block := toBlock(in.a.bits.address)
      val acq = in.a.bits.opcode.isOneOf(TLMessages.AcquireBlock, TLMessages.AcquirePerm)
      val toT = in.a.bits.param.isOneOf(TLPermissions.NtoT, TLPermissions.BtoT)
      val put = edgeIn.hasData(in.a.bits)
      snoop.bits.write := put || (acq && toT)
      val snoop_client = inIdToClientId(in.a.bits.source)

      // Track in-flight prefetches
      val tracker = RegInit(0.U(params.prefetchIds.W))
      val next_tracker = PriorityEncoder(~tracker)
      val tracker_free = !tracker(next_tracker)

      // Track arrivals
      // Trigger X for each in-flight prefetch
      val pfBase = Reg(Vec(params.prefetchIds, UInt(blockBits.W)))
      // Block for each in-flight L1 miss
      val demandBlock = Reg(Vec(edgeIn.client.endSourceId, UInt(blockBits.W)))
      when (in.a.fire && in.a.bits.opcode === TLMessages.AcquireBlock) {
        demandBlock(in.a.bits.source) := toBlock(in.a.bits.address)
      }

      val d_is_prefetch = out.d.bits.opcode === TLMessages.HintAck
      val pfFill = out.d.valid && d_is_prefetch
      val demandFill = in.d.fire && in.d.bits.opcode === TLMessages.GrantData && edgeIn.first(in.d)
      // HintAck and GrantData share the D channel
      val fill = Wire(Valid(new BOPFill(blockBits)))
      fill.valid := pfFill || demandFill
      fill.bits.prefetch := pfFill
      fill.bits.base := Mux(pfFill, pfBase(outIdToPrefetchId(out.d.bits.source)), demandBlock(in.d.bits.source))
      val fill_client = Mux(pfFill, outIdToClientId(out.d.bits.source), inIdToClientId(in.d.bits.source))

      // One prefetcher per client
      val out_arb = Module(new RRArbiter(new BOPRequest(blockBits), nClients))

      edgeOut.master.clients.zipWithIndex.foreach { case (c, i) =>
        params.prefetcher(c.name) match {
          case Some(bp) =>
            val coreId = """Core (\d+)""".r.findFirstMatchIn(c.name).map(_.group(1).toInt).getOrElse(i)
            println(s"[BOP] Prefetcher for ${c.name}: Best-Offset (${bp.offsets.size} offsets, " +
              s"max ${bp.offsets.map(_.abs).max}, page ${bp.pageBytes.map(_.toString).getOrElse("unlimited")})")
            val pf = Module(new BestOffsetPrefetcher(bp, blockBits, blockBytes, coreId))
            pf.io.snoop.valid := snoop.valid && snoop_client === i.U
            pf.io.snoop.bits := snoop.bits
            pf.io.fill.valid := fill.valid && fill_client === i.U
            pf.io.fill.bits := fill.bits
            out_arb.io.in(i).valid := pf.io.request.valid
            out_arb.io.in(i).bits := pf.io.request.bits
            pf.io.request.ready := out_arb.io.in(i).ready
          case None =>
            println(s"[BOP] Prefetcher for ${c.name}: none")
            out_arb.io.in(i).valid := false.B
            out_arb.io.in(i).bits := DontCare
        }
      }

      // Pass L1 traffic through; send prefetches when A is idle
      out <> in
      out.a.bits.source := inIdAdjuster(in.a.bits.source)
      in.b.bits.source := outIdAdjuster(out.b.bits.source)
      out.c.bits.source := inIdAdjuster(in.c.bits.source)
      in.d.valid := out.d.valid && !d_is_prefetch
      when (d_is_prefetch) { out.d.ready := true.B }
      in.d.bits.source := outIdAdjuster(out.d.bits.source)

      val hint_issued = out.a.fire && !in.a.valid
      tracker := (tracker
        ^ ((out.d.valid && d_is_prefetch) << outIdToPrefetchId(out.d.bits.source))
        ^ (hint_issued << next_tracker)
      )
      when (hint_issued) {
        pfBase(next_tracker) := out_arb.io.out.bits.base
      }

      val pf_address = toAddr(out_arb.io.out.bits.block)
      val legal_address = edgeOut.manager.findSafe(pf_address).reduce(_||_)
      val (legal, hint) = edgeOut.Hint(
        prefetchIdToOutId(next_tracker, out_arb.io.chosen),
        pf_address,
        lgBlock.U,
        Mux(out_arb.io.out.bits.write, TLHints.PREFETCH_WRITE, TLHints.PREFETCH_READ)
      )
      out_arb.io.out.ready := false.B
      when (!in.a.valid) {
        out.a.valid := out_arb.io.out.valid && tracker_free && legal && legal_address
        out.a.bits := hint
        out_arb.io.out.ready := out.a.ready
      }
      when (!legal || !legal_address) {
        out_arb.io.out.ready := true.B
      }
    }
  }
}

object BOPTLPrefetcher {
  def apply()(implicit p: Parameters) = {
    val prefetcher = LazyModule(new BOPTLPrefetcher)
    prefetcher.node
  }
}

// Puts BOPTLPrefetcher between a tile and the system bus
case class BOPTilePrefetchingMasterPortParams(tileId: Int, base: HierarchicalElementPortParamsLike)
    extends HierarchicalElementPortParamsLike {
  val where = base.where
  def injectNode(context: Attachable)(implicit p: Parameters): TLNode = {
    BOPTLPrefetcher() :*=* base.injectNode(context)(p)
  }
}

// Adds a BOP to every Rocket tile's L1 D$.
class WithBOPPrefetcher(bopParams: BestOffsetParams = BestOffsetParams(), prefetchIds: Int = 4)
    extends Config((site, here, up) => {
  case BOPTLPrefetcherKey => BOPTLPrefetcherParams(
    prefetchIds = prefetchIds,
    prefetcher = (s: String) => if (s.contains("DCache") && !s.contains("MMIO")) Some(bopParams) else None
  )
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: RocketTileAttachParams => tp.copy(crossingParams = tp.crossingParams.copy(
      master = BOPTilePrefetchingMasterPortParams(tp.tileParams.tileId, tp.crossingParams.master)))
    case other => other
  }
})
