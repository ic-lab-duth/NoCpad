#ifndef __DUTH_FUN_LIB_H__
#define __DUTH_FUN_LIB_H__

#include <systemc.h>

#ifdef HLS_CATAPULT
#include <ac_sc.h>
#include <ac_int.h>
#endif

#include "./onehot.h"

template <unsigned N> class onehot;

//============================================================================//
//============== Workaround to imitate assignments of sc types ===============//
//============================================================================//
// Functions to statically assign bit vectors to/from arrays of chars and bools.
//   Used as e workaround to avoid variables in .range() function of bit vectors
template<class T, int BYTES> struct duth_fun {
  static inline void assign_char2bv(T& lhs, unsigned char *rhs) {
    duth_fun<T, BYTES-1>::assign_char2bv(lhs, rhs);
    lhs.range((BYTES*8)-1, ((BYTES-1)*8)) = rhs[BYTES-1];
  }
  
  static inline void assign_char2ac(T& lhs, unsigned char *rhs) {
    duth_fun<T, BYTES-1>::assign_char2ac(lhs, rhs);
    lhs.set_slc((BYTES-1)*8, (ac_int<8, false>) rhs[BYTES-1]);
  }
  
  static inline void assign_bv2char(unsigned char *lhs, T& rhs) {
    duth_fun<T, BYTES-1>::assign_bv2char(lhs, rhs);
    lhs[BYTES-1] = rhs.range((BYTES*8)-1, ((BYTES-1)*8)).to_uint();
  }
  
  static inline void assign_ac2char(unsigned char *lhs, T& rhs) {
    duth_fun<T, BYTES-1>::assign_ac2char(lhs, rhs);
    lhs[BYTES-1] = (rhs.template slc<8>((BYTES-1)*8));
  }
  
  static inline void assign_bool2bv(T& lhs, bool *rhs) {
    duth_fun<T, BYTES-1>::assign_bool2bv(lhs, rhs);
    lhs.range(BYTES-1, BYTES-1) = rhs[BYTES-1];
  }
  
  static inline void assign_bv2bool(bool *lhs, T& rhs) {
    duth_fun<T, BYTES-1>::assign_bv2bool(lhs, rhs);
    lhs[BYTES-1] = rhs.range(BYTES-1, BYTES-1).to_uint();
  }
  
  static inline void assign_bool2ac(T& lhs, bool *rhs) {
    duth_fun<T, BYTES-1>::assign_bool2ac(lhs, rhs);
    lhs.set_slc(BYTES-1, (ac_int<1, false>) rhs[BYTES-1]);
  }
  
  static inline void assign_ac2bool(bool *lhs, T& rhs) {
    duth_fun<T, BYTES-1>::assign_ac2bool(lhs, rhs);
    lhs[BYTES-1] = rhs.template slc<1>(BYTES-1);
  }
};
template<class T> struct duth_fun<T, 1> {
  static inline void assign_char2bv(T& lhs, unsigned char *rhs) {
    lhs.range(7, 0) = rhs[0];
  }
  
  static inline void assign_char2ac(T& lhs, unsigned char *rhs) {
    lhs.set_slc(0, (ac_int<8, false>) rhs[0]);
  }
  
  static inline void assign_bv2char(unsigned char *lhs, T& rhs) {
    lhs[0] = rhs.range(7, 0).to_uint();
  }
  
  static inline void assign_ac2char(unsigned char *lhs, T& rhs) {
    lhs[0] = rhs.template slc<8>(0);
  }
  
  static inline void assign_bool2bv(T& lhs, bool *rhs) {
    lhs.range(0, 0) = rhs[0];
  }
  
  static inline void assign_bv2bool(bool *lhs, T& rhs) {
    lhs[0] = rhs.range(0, 0).to_uint();
  }
  
  static inline void assign_bool2ac(T& lhs, bool *rhs) {
    lhs.set_slc(0, (ac_int<1, false>) rhs[0]);
  }
  
  static inline void assign_ac2bool(bool *lhs, T& rhs) {
    lhs[0] = rhs.template slc<1>(0);
  }
};

//============================================================================//
//==================== Statically calculate Number of bits ===================//
//============================================================================//
template <int x>
struct clog2 { enum { val = 1 + clog2<(x>>1)>::val }; };

template <> struct clog2<1> { enum { val = 1 }; };

//============================================================================//
//============ Weighted-Binary to One-Hot conversion (case based) ============//
//============================================================================//
// ISSUES ON CO-SIMULATION
/*
template<int OH_BITS>
sc_uint<OH_BITS> wb2oh_case (const sc_uint< clog2<OH_BITS>::val > wb_i);

template <>
sc_uint<1> wb2oh_case (const sc_uint< 1 > wb_i) {
  return 1;
};

template <> 
sc_uint<2> wb2oh_case (const sc_uint< clog2<2>::val > wb_i) {
  sc_uint<2> oh_rep;
  switch (wb_i) {
    case 0 : oh_rep = 1;
      break;
    case 1 : oh_rep = 2;
      break;
    default : oh_rep = 1;
      break;
  }
  return oh_rep;
};

template <> 
sc_uint<3> wb2oh_case (const sc_uint< clog2<3>::val > wb_i) {
  sc_uint<3> oh_rep;
  switch (wb_i) {
    case 0 : oh_rep = 1;
      break;
    case 1 : oh_rep = 2;
      break;
    case 2 : oh_rep = 4;
      break;
    case 3 : oh_rep = 8; // Possible, but functionally wrong 
      break;
    default : oh_rep = 1;
      break;
  }
  return oh_rep;
};

template <> 
sc_uint<4> wb2oh_case (const sc_uint< clog2<4>::val > wb_i) {
  sc_uint<4> oh_rep;
  switch (wb_i) {
    case 0 : oh_rep = 1;
      break;
    case 1 : oh_rep = 2;
      break;
    case 2 : oh_rep = 4;
      break;
    case 3 : oh_rep = 8;
      break;
    default : oh_rep = 1;
      break;
  }
  return oh_rep;
};

template <> 
sc_uint<5> wb2oh_case (const sc_uint< clog2<5>::val > wb_i) {
  sc_uint<5> oh_rep;
  switch (wb_i) {
    case 0 : oh_rep = 1;
      break;
    case 1 : oh_rep = 2;
      break;
    case 2 : oh_rep = 4;
      break;
    case 3 : oh_rep = 8; 
      break;
    case 4 : oh_rep = 16; 
      break;
    case 5 : oh_rep = 32; // Should not occur
      break;
    case 6 : oh_rep = 64; // Should not occur
      break;
    case 7 : oh_rep = 128; // Should not occur
      break;
    default : oh_rep = 1;
      break;
  }
  return oh_rep;
};

template <> 
sc_uint<6> wb2oh_case (const sc_uint< clog2<6>::val > wb_i) {
  sc_uint<6> oh_rep;
  switch (wb_i) {
    case 0 : oh_rep = 1;
      break;
    case 1 : oh_rep = 2;
      break;
    case 2 : oh_rep = 4;
      break;
    case 3 : oh_rep = 8; 
      break;
    case 4 : oh_rep = 16; 
      break;
    case 5 : oh_rep = 32;
      break;
    case 6 : oh_rep = 64;  // Should not occur
      break;
    case 7 : oh_rep = 128; // Should not occur
      break;
    default : oh_rep = 1;
      break;
  }
  return oh_rep;
};

template <>
sc_uint<7> wb2oh_case (const sc_uint< clog2<7>::val > wb_i) {
  sc_uint<7> oh_rep;
  switch (wb_i) {
    case 0 : oh_rep = 1;
      break;
    case 1 : oh_rep = 2;
      break;
    case 2 : oh_rep = 4;
      break;
    case 3 : oh_rep = 8;
      break;
    case 4 : oh_rep = 16;
      break;
    case 5 : oh_rep = 32;
      break;
    case 6 : oh_rep = 64;
      break;
    case 7 : oh_rep = 128; // Should not occur
      break;
    default : oh_rep = 1;
      break;
  }
  return oh_rep;
};

template <>
sc_uint<8> wb2oh_case (const sc_uint< clog2<8>::val > wb_i) {
  sc_uint<8> oh_rep;
  switch (wb_i) {
    case 0 : oh_rep = 1;
      break;
    case 1 : oh_rep = 2;
      break;
    case 2 : oh_rep = 4;
      break;
    case 3 : oh_rep = 8;
      break;
    case 4 : oh_rep = 16;
      break;
    case 5 : oh_rep = 32;
      break;
    case 6 : oh_rep = 64;
      break;
    case 7 : oh_rep = 128;
      break;
    default : oh_rep = 1;
      break;
  }
  return oh_rep;
};
*/
//============================================================================//
//============================== Mux Container Struct ========================//
//============================================================================//
template <class T, int SIZE> struct mux {
  static T mux_oh_case(const sc_uint<SIZE> sel_i, const T data_i[SIZE]);
  static T mux_oh_case(const onehot<SIZE> sel_i, const T data_i[SIZE]);
  static T mux_oh_ao(const sc_uint<SIZE> sel_i, const T data_i[SIZE]);
};
//============================================================================//
//======================== One-Hot Multiplexer (case based) ==================//
//============================================================================//
template <class T>
struct mux<T, 1> {
  static T mux_oh_case (const sc_uint<1> sel_i, const T data_i[1] ) {
    return data_i[0];
  };
  
  static T mux_oh_case (const onehot<1> sel_i, const T data_i[1] ) {
    return data_i[0];
  };
  
  static T mux_oh_ao (const sc_uint<1> sel_i, const T data_i[1] ) {
    return data_i[0];
  };
};

template <class T>
struct mux<T, 2> {
  static T mux_oh_case (const sc_uint<2> sel_i, const T data_i[2] ) {
    T selected;
    switch (sel_i) {
      case 1 : selected = data_i[0];
        break;
      case 2 : selected = data_i[1];
        break;
      default : selected = data_i[0];
        break;
    }
    return selected;
  };
  
  static T mux_oh_case (const onehot<2> sel_i, const T data_i[2] ) {
    T selected;
    switch (sel_i.val) {
      case 1 : selected = data_i[0];
        break;
      case 2 : selected = data_i[1];
        break;
      default : selected = data_i[0];
        break;
    }
    return selected;
  };
  
  static T mux_oh_ao (const sc_uint<2> sel_i, const T data_i[2] ) {
    T selected = T();
    #pragma hls_unroll yes
    for(int i=0; i<2; ++i) {
      bool cur_sel_bit = (sel_i >> i) & 1;
      selected = selected | data_i[i].and_mask(cur_sel_bit);
    }
    return selected;
  };
};


template <class T>
struct mux<T, 3> {
  static T mux_oh_case (const sc_uint<3> sel_i, const T data_i[3] ) {
    T selected;
    switch (sel_i) {
      case 1 : selected = data_i[0];
        break;
      case 2 : selected = data_i[1];
        break;
      case 4 : selected = data_i[2];
        break;
      default : selected = data_i[0];
        break;
    }
    return selected;
  };
  
  static T mux_oh_case (const onehot<3> sel_i, const T data_i[3] ) {
    T selected;
    switch (sel_i.val) {
      case 1 : selected = data_i[0];
        break;
      case 2 : selected = data_i[1];
        break;
      case 4 : selected = data_i[2];
        break;
      default : selected = data_i[0];
        break;
    }
    return selected;
  };
  
  static T mux_oh_ao (const sc_uint<3> sel_i, const T data_i[3] ) {
    T selected = T();
    #pragma hls_unroll yes
    for(int i=0; i<3; ++i) {
      bool cur_sel_bit = (sel_i >> i) & 1;
      selected = selected | data_i[i].and_mask(cur_sel_bit);
    }
    return selected;
  };
};

template <class T>
struct mux<T, 4> {
  static T mux_oh_case (const sc_uint<4> sel_i, const T data_i[4] ) {
    T selected;
    switch (sel_i) {
      case 1 : selected = data_i[0];
        break;
      case 2 : selected = data_i[1];
        break;
      case 4 : selected = data_i[2];
        break;
      case 8 : selected = data_i[3];
        break;
      default : selected = data_i[0];
        break;
    }
    return selected;
  };
  
  static T mux_oh_case (const onehot<4> sel_i, const T data_i[4] ) {
    T selected;
    switch (sel_i.val) {
      case 1 : selected = data_i[0];
        break;
      case 2 : selected = data_i[1];
        break;
      case 4 : selected = data_i[2];
        break;
      case 8 : selected = data_i[3];
        break;
      default : selected = data_i[0];
        break;
    }
    return selected;
  };
  
  static T mux_oh_ao (const sc_uint<4> sel_i, const T data_i[4] ) {
    T selected = T();
    #pragma hls_unroll yes
    for(int i=0; i<4; ++i) {
      bool cur_sel_bit = (sel_i >> i) & 1;
      selected = selected | data_i[i].and_mask(cur_sel_bit);
    }
    return selected;
  };
};

template <class T>
struct mux<T, 5> {
  static T mux_oh_case (const sc_uint<5> sel_i, const T data_i[5] ) {
    T selected;
    switch (sel_i) {
      case 1 : selected = data_i[0];
        break;
      case 2 : selected = data_i[1];
        break;
      case 4 : selected = data_i[2];
        break;
      case 8 : selected = data_i[3];
        break;
      case 16 : selected = data_i[4];
        break;
      default : selected = data_i[0];
        break;
    }
    return selected;
  };
  
  static T mux_oh_case (const onehot<5> sel_i, const T data_i[5] ) {
    T selected;
    switch (sel_i.val) {
      case 1 : selected = data_i[0];
        break;
      case 2 : selected = data_i[1];
        break;
      case 4 : selected = data_i[2];
        break;
      case 8 : selected = data_i[3];
        break;
      case 16 : selected = data_i[4];
        break;
      default : selected = data_i[0];
        break;
    }
    return selected;
  };
  
  static T mux_oh_ao (const sc_uint<5> sel_i, const T data_i[5] ) {
    T selected = T();
    #pragma hls_unroll yes
    for(int i=0; i<5; ++i) {
      bool cur_sel_bit = (sel_i >> i) & 1;
      selected = selected | data_i[i].and_mask(cur_sel_bit);
    }
    return selected;
  };
};

template <class T>
struct mux<T, 6> {
  static T mux_oh_case (const sc_uint<6> sel_i, const T data_i[6] ) {
    T selected;
    switch (sel_i) {
      case 1 : selected = data_i[0];
        break;
      case 2 : selected = data_i[1];
        break;
      case 4 : selected = data_i[2];
        break;
      case 8 : selected = data_i[3];
        break;
      case 16 : selected = data_i[4];
        break;
      case 32 : selected = data_i[5];
        break;
      default : selected = data_i[0];
        break;
    }
    return selected;
  };
  
  static T mux_oh_case (const onehot<6> sel_i, const T data_i[6] ) {
    T selected;
    switch (sel_i.val) {
      case 1 : selected = data_i[0];
        break;
      case 2 : selected = data_i[1];
        break;
      case 4 : selected = data_i[2];
        break;
      case 8 : selected = data_i[3];
        break;
      case 16 : selected = data_i[4];
        break;
      case 32 : selected = data_i[5];
        break;
      default : selected = data_i[0];
        break;
    }
    return selected;
  };
  
  static T mux_oh_ao (const sc_uint<6> sel_i, const T data_i[6] ) {
    T selected = T();
    #pragma hls_unroll yes
    for(int i=0; i<6; ++i) {
      bool cur_sel_bit = (sel_i >> i) & 1;
      selected = selected | data_i[i].and_mask(cur_sel_bit);
    }
    return selected;
  };
};

template <class T>
struct mux<T, 7> {
    static T mux_oh_case (const sc_uint<7> sel_i, const T data_i[7] ) {
      T selected;
      switch (sel_i) {
        case 1 : selected = data_i[0];
          break;
        case 2 : selected = data_i[1];
          break;
        case 4 : selected = data_i[2];
          break;
        case 8 : selected = data_i[3];
          break;
        case 16 : selected = data_i[4];
          break;
        case 32 : selected = data_i[5];
          break;
        case 64 : selected = data_i[6];
          break;
        default : selected = data_i[0];
          break;
      }
      return selected;
    };
    
    static T mux_oh_case (const onehot<7> sel_i, const T data_i[7] ) {
      T selected;
      switch (sel_i.val) {
        case 1 : selected = data_i[0];
          break;
        case 2 : selected = data_i[1];
          break;
        case 4 : selected = data_i[2];
          break;
        case 8 : selected = data_i[3];
          break;
        case 16 : selected = data_i[4];
          break;
        case 32 : selected = data_i[5];
          break;
        case 64 : selected = data_i[6];
          break;
        default : selected = data_i[0];
          break;
      }
      return selected;
    };
    
    static T mux_oh_ao (const sc_uint<7> sel_i, const T data_i[7] ) {
      T selected = T();
#pragma hls_unroll yes
      for(int i=0; i<7; ++i) {
        bool cur_sel_bit = (sel_i >> i) & 1;
        selected = selected | data_i[i].and_mask(cur_sel_bit);
      }
      return selected;
    };
};

template <class T>
struct mux<T, 8> {
    static T mux_oh_case (const sc_uint<8> sel_i, const T data_i[8] ) {
      T selected;
      switch (sel_i) {
        case 1 : selected = data_i[0];
          break;
        case 2 : selected = data_i[1];
          break;
        case 4 : selected = data_i[2];
          break;
        case 8 : selected = data_i[3];
          break;
        case 16 : selected = data_i[4];
          break;
        case 32 : selected = data_i[5];
          break;
        case 64 : selected = data_i[6];
          break;
        case 128 : selected = data_i[7];
          break;
        default : selected = data_i[0];
          break;
      }
      return selected;
    };
    
    static T mux_oh_case (const onehot<8> sel_i, const T data_i[8] ) {
      T selected;
      switch (sel_i.val) {
        case 1 : selected = data_i[0];
          break;
        case 2 : selected = data_i[1];
          break;
        case 4 : selected = data_i[2];
          break;
        case 8 : selected = data_i[3];
          break;
        case 16 : selected = data_i[4];
          break;
        case 32 : selected = data_i[5];
          break;
        case 64 : selected = data_i[6];
          break;
        case 128 : selected = data_i[7];
          break;
        default : selected = data_i[0];
          break;
      }
      return selected;
    };
    
    static T mux_oh_ao (const sc_uint<8> sel_i, const T data_i[8] ) {
      T selected = T();
#pragma hls_unroll yes
      for(int i=0; i<8; ++i) {
        bool cur_sel_bit = (sel_i >> i) & 1;
        selected = selected | data_i[i].and_mask(cur_sel_bit);
      }
      return selected;
    };
};

//============================================================================//
//==================== Swap Dimensions of an Array Class =====================//
//============================================================================//
template<class X, int X_SZ, class Y, int Y_SZ>
inline void swap_dim (const X in[X_SZ], Y out[Y_SZ]) {
#pragma hls_unroll yes
  for(int i=0; i<Y_SZ; ++i) {
    Y mule = 0;
#pragma hls_unroll yes
    for(int j=0; j<X_SZ; ++j) {
      mule = mule | (((in[j] >> i) & 1) << j );
    }
    out[i] = mule;
  }
};

template<class X, class Y>
inline void swap_dim (const X in[X::WIDTH], Y out[Y::WIDTH]) {
#pragma hls_unroll yes
  for(int i=0; i<Y::WIDTH; ++i) {
    Y mule;
#pragma hls_unroll yes
    for(int j=0; j<X::WIDTH; ++j) {
      mule.val |= (((in[j].val >> i) & 1) << j );
    }
    out[i] = mule;
  }
};

//============================================================================//
//============================== One-Hot Credit Class ========================//
//============================================================================//
template <int N>
class credit_class {
public:
  sc_uint<N+1> credits_oh = (1<<N);
  
  inline bool ready() {return !credits_oh[0];};
  
  inline void incr() { credits_oh = (credits_oh << 1);};
  inline void decr() { credits_oh = (credits_oh >> 1);};
};




#endif // __DUTH_FUN_LIB_H__
