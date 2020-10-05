#ifndef __DNP20_V0_DEF__
#define __DNP20_V0_DEF__


// Definition of Duth Network Protocol.
//   Interconnect's internal packetization protocol 
namespace dnp {
    enum {
      PHIT_W = 24, // Phit Width
      
      V_W = 2, // Virtual Channel
      S_W = 4, // Source
      D_W = 4, // Destination
      Q_W = 3, // QoS
      T_W = 2, // Type
      
      V_PTR = 0,
      S_PTR = (V_PTR + V_W),
      D_PTR = (S_PTR + S_W),
      Q_PTR = (D_PTR + D_W),
      T_PTR = (Q_PTR + Q_W),
      
      // AXI RELATED WIDTHS
      ID_W = 4, // AXI Transaction ID
      BU_W = 2, // AXI Burst
      SZ_W = 3, // AXI Size
      LE_W = 8, // AXI Length
      AL_W = 16, // Address Low
      AH_W = 16, // Address High
      AP_W = 8, // Address part (for alignment)
      RE_W = 2, // AXI Write Responce
      REORD_W = 3, // Ticket for reorder buffer
  
      B_W  = 8, // Byte Width ...
      E_W  = 1, // Enable width
      LA_W = 1, // AXI Last
    };
  
  // Read and Write Request field pointers
  struct req {
    enum {
      ID_PTR = T_PTR+T_W,
      REORD_PTR = ID_PTR+ID_W,
      
      AL_PTR = 0,
      LE_PTR = AL_PTR+AL_W,
      
      AH_PTR = 0,
      SZ_PTR = AH_PTR+AH_W,
      BU_PTR = SZ_PTR+SZ_W,
    };
  };
  
  // Write Responce field pointers
  struct wresp {
    enum {
      ID_PTR    = T_PTR+T_W,
      REORD_PTR = ID_PTR+ID_W,
      RESP_PTR  = REORD_PTR+REORD_W,
    };
  };
  
  // Read Responce field pointers
  struct rresp {
    enum {
      ID_PTR    = T_PTR+T_W,
      REORD_PTR = ID_PTR+ID_W,
      BU_PTR    = REORD_PTR+REORD_W,
    
      SZ_PTR = 0,
      LE_PTR = SZ_PTR+SZ_W,
      AP_PTR = LE_PTR+LE_W,
    };
  };
  
  // Write request Data field pointers
  struct wdata {
    enum {
      B0_PTR = 0,
      B1_PTR = B0_PTR+B_W,
      E0_PTR = B1_PTR+B_W,
      E1_PTR = E0_PTR+E_W,
      LA_PTR = E1_PTR+E_W,
    };
  };
  
  // Read response Data field pointers
  struct rdata {
    enum {
      B0_PTR = 0,
      B1_PTR = B0_PTR+B_W,
      RE_PTR = B1_PTR+B_W,
      LA_PTR = RE_PTR+RE_W,
    };
  };
  
  enum PACK_TYPE {
    PACK_TYPE__WR_REQ  = 0,
    PACK_TYPE__WR_RESP = 1,
    PACK_TYPE__RD_REQ  = 2,
    PACK_TYPE__RD_RESP = 3
  };
  
}

#endif // __DNP20_V0_DEF__
