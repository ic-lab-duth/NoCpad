#ifndef __ONEHOT_CLASS__
#define __ONEHOT_CLASS__

#include <systemc.h>
#include "./duth_fun.h"

//============================================================================//
//============================== One-Hot Class ========================//
//============================================================================//
template <unsigned N>
class onehot {
public:
  static const unsigned WIDTH = N;
  sc_uint<N> val;
  
  onehot() {onehot(1);};
  onehot(unsigned init_val) {val = init_val;};
  
  // Only for SystemC
  inline friend void sc_trace(sc_trace_file* tf, const onehot& oh_val, const std::string& name) {
    sc_trace(tf, oh_val.val, name + ".val");
  }
  
  template<typename T>
  inline void set(T wb_val) {
    switch (wb_val) {
      case 0 : val = (1<<0);
        break;
      case 1 : val = (1<<1);
        break;
      case 2 : val = (1<<2);
        break;
      case 3 : val = (1<<3);
        break;
      case 4 : val = (1<<4);
        break;
      case 5 : val = (1<<5);
        break;
      case 6 : val = (1<<6);
        break;
      case 7 : val = (1<<7);
        break;
      default : val = (1<<0);;
        break;
    }
  }; //(1<<wb_val);};
  
  template<unsigned RHS_N>
  inline void set(onehot<RHS_N> oh_val) { val = oh_val.val;};
  
  inline sc_uint<N> get() {return val;};
  
  inline bool is_ready()  {return !val[0];};
  inline bool or_reduce() {return val.or_reduce();};
  
  inline onehot<N> and_mask(bool bit) const {
    onehot<N> mask(0);
    #pragma hls_unroll yes
    for(int i=0; i<N; ++i) mask.val[i] |= (bit << i);  // Build the mask
    
    return onehot<N>(val & mask.val);
  };
  
  inline void increase() { val = (val << 1);};
  inline void decrease() { val = (val >> 1);};
  
  template <class T>
  T mux(const T data_i[N]);
  
  //inline bool operator[] (const unsigned pos) {
  //  return ((val>>pos) & 1);
  //z};
  
  sc_dt::sc_uint_bitref&         operator [] ( unsigned pos) {return val[pos];};
  const sc_dt::sc_uint_bitref_r& operator [] ( unsigned pos ) const {return val[pos];};
  
  //template<typename T>
  //bool operator== (const T &rhs) {
  //  return (val==rhs);
  //};
  
  template<typename T>
  onehot& operator= (const T &rhs) {
    this->set(rhs);
  };
};

template<>
template<class T>
T onehot<2>::mux(const T data_i[2] ) {
  T selected;
  switch (val) {
    case 1 : selected = data_i[0];
      break;
    case 2 : selected = data_i[1];
      break;
    default : selected = data_i[0];
      break;
  }
  return selected;
};

template<>
template<class T>
T onehot<3>::mux(const T data_i[3] ) {
  T selected;
  switch (val) {
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

template<>
template<class T>
T onehot<4>::mux(const T data_i[4] ) {
  T selected;
  switch (val) {
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

template<>
template<class T>
T onehot<5>::mux(const T data_i[5] ) {
  T selected;
  switch (val) {
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

template<>
template<class T>
T onehot<6>::mux(const T data_i[6] ) {
  T selected;
  switch (val) {
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

#endif // __ONEHOT_CLASS__
