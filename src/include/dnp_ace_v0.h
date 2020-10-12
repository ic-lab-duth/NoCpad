#ifndef __DNP_ACE_DEF__
#define __DNP_ACE_DEF__


// Definition of Duth Network Protocol for ACE network.
//   Interconnect's internal packetization protocol 
namespace dnp {
  enum {
    // !!!! THIS MUST BE 24. 20 is temp for router synth!!!
    PHIT_W = 24, // Phit Width

    V_W = 2, // Virtual Channel
    S_W = 3, // Source
    D_W = 3, // Destination
    Q_W = 3, // QoS
    T_W = 3, // Type

    V_PTR = 0,
    S_PTR = (V_PTR + V_W),
    D_PTR = (S_PTR + S_W),
    Q_PTR = (D_PTR + D_W),
    T_PTR = (Q_PTR + Q_W),
  };
  
  class ace {
    public:
    
    enum {
      // AXI RELATED WIDTHS
      ID_W = 4, // AXI Transaction ID
      BU_W = 2, // AXI Burst
      SZ_W = 3, // AXI Size
      LE_W = 8, // AXI Length
      AL_W = 16, // Address Low
      AH_W = 16, // Address High
      AP_W = 8, // Address part (for alignment)
      W_RE_W = 2, // AXI Write Response
      R_RE_W = 4, // ACE Read response
  
      B_W = 8, // Byte Width ...
      E_W = 1, // Enable width
      LA_W = 1, // AXI Last
  
      // ACE RELATED WIDTHS
      SNP_W = 4,
      DOM_W = 2,
      BAR_W = 2,
      UNQ_W = 1,
  
      C_PROT_W = 3,
      C_RESP_W = 5,
      C_HAS_DATA_W = 1,
    };
    
    struct req {
      enum {
        // PHIT #0
        ID_PHIT = 0,
        DOM_PHIT = 0,
        SNP_PHIT = 0,
  
        ID_PTR  = T_PTR+T_W,
        DOM_PTR = ID_PTR + ID_W,
        SNP_PTR = DOM_PTR + DOM_W,
        // PHIT #1
        AL_PHIT = 1,
        LE_PHIT = 1,
        
        AL_PTR = 0,
        LE_PTR = AL_PTR+AL_W,
        // PHIT #2
        AH_PHIT  = 2,
        SZ_PHIT  = 2,
        BU_PHIT  = 2,
        BAR_PHIT = 2,
        UNQ_PHIT = 2,
        
        AH_PTR  = 0,
        SZ_PTR  = AH_PTR+AH_W,
        BU_PTR  = SZ_PTR+SZ_W,
        BAR_PTR = BU_PTR+BU_W,
        UNQ_PTR = BAR_PTR+BAR_W,
      };
    };
    
    struct wresp {
      enum {
        // PHIT #0
        ID_PHIT = 0,
        RESP_PHIT = 0,
        
        ID_PTR   = T_PTR+T_W,
        RESP_PTR = ID_PTR+ID_W,
      };
    };
    
    struct rresp {
      enum {
        // PHIT #0
        ID_PHIT = 0,
        BU_PHIT = 0,
        
        ID_PTR = T_PTR+T_W,
        BU_PTR = ID_PTR+ID_W,
        // PHIT #1
        SZ_PHIT = 1,
        LE_PHIT = 1,
        AP_PHIT = 1,
        
        SZ_PTR = 0,
        LE_PTR = SZ_PTR+SZ_W,
        AP_PTR = LE_PTR+LE_W,
      };
    };
    
    struct wdata {
      enum {
        B0_PTR = 0,
        B1_PTR = B0_PTR+B_W,
        LA_PTR = B1_PTR+B_W,
        E0_PTR = LA_PTR+LA_W,
        E1_PTR = E0_PTR+E_W,
      };
    };
    
    struct rdata {
      enum {
        B0_PTR = 0,
        B1_PTR = B0_PTR+B_W,
        LA_PTR = B1_PTR+B_W,
        RE_PTR = LA_PTR+LA_W,
      };
    };
    
    // ACE Extension
    struct creq {
      enum {
        // PHIT #1
        AL_PHIT  = 1,
        SNP_PHIT = 1,
        
        AL_PTR  = 0,
        SNP_PTR = AL_PTR+AL_W,
        // PHIT #2
        AH_PHIT = 2,
        C_PROT_PHIT = 2,
        
        AH_PTR = 0,
        C_PROT_PTR = AH_PTR+AH_W,
      };
    };
    
    struct cresp {
      enum {
        // PHIT #0
        C_RESP_PHIT     = 0,
        C_HAS_DATA_PHIT = 0,
        
        C_RESP_PTR     = T_PTR+T_W,
        C_HAS_DATA_PTR = C_RESP_PTR + C_RESP_W,
      };
    };
  }; // class ACE
  
  enum PACK_TYPE {
    PACK_TYPE__WR_REQ  = 0,
    PACK_TYPE__WR_RESP = 1,
    PACK_TYPE__RD_REQ  = 2,
    PACK_TYPE__RD_RESP = 3,
    PACK_TYPE__C_RD_REQ   = 4,
    PACK_TYPE__C_RD_RESP  = 5,
    PACK_TYPE__C_WR_REQ   = 6,
    PACK_TYPE__C_WR_RESP  = 7
    //PACK_TYPE__SNP_REQ   = ?
    //PACK_TYPE__SNP_RESP  = ?
  };
}; // namespace dnp

#endif // __DNP_ACE_DEF__
