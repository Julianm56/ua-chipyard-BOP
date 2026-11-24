package chipyard

import org.chipsalliance.cde.config.Config

// Build: cd /workspace/chipyard/sims/verilator && make CONFIG=BOPRocketConfig

class BOPEvalBaseConfig extends Config(
  new chipyard.config.WithNPerfCounters(8) ++
  new freechips.rocketchip.rocket.WithL1DCacheNonblocking(2) ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig
)

// Paper settings, 4 KiB page limit
class BOPRocketConfig extends Config(
  new bop.WithBOPPrefetcher(bop.BestOffsetParams()) ++
  new BOPEvalBaseConfig
)

// No page limit, offsets up to 256
class BOPNoPageLimitRocketConfig extends Config(
  new bop.WithBOPPrefetcher(bop.BestOffsetParams(
    offsets = bop.BestOffsetParams.smoothOffsets(maxOffset = 256),
    pageBytes = None
  )) ++
  new BOPEvalBaseConfig
)
