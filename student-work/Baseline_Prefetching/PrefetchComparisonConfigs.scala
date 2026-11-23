package chipyard

import org.chipsalliance.cde.config.Config

// Shared processor settings and TileLink prefetcher attachment.
class PrefetchComparisonBaseConfig extends Config(
  new chipyard.config.WithTilePrefetchers ++
  new freechips.rocketchip.rocket.WithL1DCacheNonblocking(2) ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig
)

// Baseline: keep the attachment, but generate no prefetch requests.
class NoPrefetchRocketConfig extends Config(
  new barf.WithTLDCachePrefetcher(
    barf.NullPrefetcherParams()
  ) ++
  new PrefetchComparisonBaseConfig
)

// NextLine data prefetching.
class NextLineRocketConfig extends Config(
  new barf.WithTLDCachePrefetcher(
    barf.SingleNextLinePrefetcherParams()
  ) ++
  new PrefetchComparisonBaseConfig
)

// Strided data prefetching.
class StridedRocketConfig extends Config(
  new barf.WithTLDCachePrefetcher(
    barf.SingleStridedPrefetcherParams()
  ) ++
  new PrefetchComparisonBaseConfig
)

// AMPM data prefetching.
class AMPMRocketConfig extends Config(
  new barf.WithTLDCachePrefetcher(
    barf.SingleAMPMPrefetcherParams()
  ) ++
  new PrefetchComparisonBaseConfig
)