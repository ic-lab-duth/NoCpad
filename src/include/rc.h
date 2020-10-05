#ifndef __ROUTING_COMPUTATION_HEADER__
#define __ROUTING_COMPUTATION_HEADER__

#include <systemc.h>
#include "./dnp_ace_v0.h"

enum rc_t {RC_DIRECT, RC_XY, RC_FIXED, RC_TYPE, RC_COMMON};

// Direct RC : The Dst Node is the Output port
//template<rc_t TYPE, typename...T >
//inline unsigned char do_rc(T...params);

template<rc_t TYPE, typename...T >
inline unsigned char do_rc(T...params);

template<>
inline unsigned char do_rc<RC_DIRECT, sc_uint<dnp::ace::D_W> > (sc_uint<dnp::ace::D_W> destination)     {return destination;};

// TYPE RC : Used for splitter/mergers. Routes RD to Out==0, and WR to Out==1
template<>
inline unsigned char do_rc<RC_TYPE, sc_lv<dnp::ace::T_W> >   (sc_lv<dnp::ace::T_W> type) {return (type==dnp::PACK_TYPE__RD_REQ || type==dnp::PACK_TYPE__RD_RESP) ? 0 : 1;};

// Const RC : Used for mergers to always request port #0
template<>
inline unsigned char do_rc<RC_FIXED>  () {return 0;};

// Const RC : Used for mergers to always request port #0
template<>
inline unsigned char do_rc<RC_COMMON, sc_lv<dnp::ace::D_W>, sc_lv<dnp::ace::T_W> >  (sc_lv<dnp::ace::D_W> destination, sc_lv<dnp::ace::T_W> type) {
  if (type==dnp::PACK_TYPE__RD_REQ || type==dnp::PACK_TYPE__RD_RESP) return destination.to_uint();
  else                                                               return destination.to_uint()+2;
};
// LUT RC : the Outport is selected from a LUT
//inline unsigned char do_rc_lut (sc_lv<dnp::D_W> dst) {return route_lut[dst];};

#endif // __ROUTING_COMPUTATION_HEADER__
