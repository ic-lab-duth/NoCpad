#ifndef __AXI_CONFIG_DUTH_H__
#define __AXI_CONFIG_DUTH_H__

namespace axi {

// Extension of Matchlib AXI configuration
namespace cfg {
  /**
   * \brief A standard AXI configuration with SIZE field.
   */
  struct standard_duth {
    enum {
      useACE = 0,
      dataWidth = 64,
      useVariableBeatSize = 1,
      useMisalignedAddresses = 0,
      useLast = 1,
      useWriteStrobes = 1,
      useBurst = 1, useFixedBurst = 1, useWrapBurst = 0, maxBurstSize = 256,
      useQoS = 0, useLock = 0, useProt = 0, useCache = 0, useRegion = 0,
      aUserWidth = 0, wUserWidth = 0, bUserWidth = 0, rUserWidth = 0,
      addrWidth = 32,
      idWidth = 4,
      useWriteResponses = 1,
    };
  };

  struct standard_duth_128 {
    enum {
      useACE = 0,
      dataWidth = 128,
      useVariableBeatSize = 1,
      useMisalignedAddresses = 0,
      useLast = 1,
      useWriteStrobes = 1,
      useBurst = 1, useFixedBurst = 1, useWrapBurst = 0, maxBurstSize = 256,
      useQoS = 0, useLock = 0, useProt = 0, useCache = 0, useRegion = 0,
      aUserWidth = 0, wUserWidth = 0, bUserWidth = 0, rUserWidth = 0,
      addrWidth = 32,
      idWidth = 4,
      useWriteResponses = 1,
    };
  };
  /**
   * \brief ACE enabled, AXI configuration.
   */
  struct ace {
    enum {
      useACE    = 1,
      CacheLineWidth = 64, // bits
      dataWidth = 64,
      useVariableBeatSize = 1,
      useMisalignedAddresses = 0,
      useLast = 1,
      useWriteStrobes = 1,
      useBurst = 1, useFixedBurst = 1, useWrapBurst = 0, maxBurstSize = 256,
      useQoS = 0, useLock = 0, useProt = 0, useCache = 0, useRegion = 0,
      aUserWidth = 0, wUserWidth = 0, bUserWidth = 0, rUserWidth = 0,
      addrWidth = 32,
      idWidth = 4,
      useWriteResponses = 1,
    };
  };
}; // namespace cfg
}; // namespace axi

#endif
