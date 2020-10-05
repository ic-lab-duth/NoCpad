#ifndef __FIFO_QUEUE__
#define __FIFO_QUEUE__

#include "../include/duth_fun.h"
#include "../include/nvhls_assert.h"

template <typename T, unsigned SIZE>
class fifo_queue {
  public :
  //typedef sc_uint< clog2<SIZE>::val > wb_depth_t;
  T  mem[SIZE];
  
  sc_uint<SIZE> push_ptr;
  sc_uint<SIZE> pop_ptr;
  
  sc_uint<SIZE+1> item_count;
  
  public :
  fifo_queue(){
    reset();
  };
  
  void reset () {
    push_ptr = 1; // points [0] in one hot
    pop_ptr  = 1; // points [0] in one hot
    
    item_count = 1; // empty buffer when item_count[0]
                    // full buffer  when item_count[SIZE]
  };
  
  // Non Intrusive
  inline bool full()  const {return (item_count[SIZE]);};
  inline bool ready() const {return !full();};
  
  inline bool empty() const {return (item_count[0]);};
  inline bool valid() const {return !empty();};
  
  inline T peek()     const {
    //mem[pop_ptr];
    return mux<T, SIZE>::mux_oh_case(pop_ptr, mem);
    //return mux<T, SIZE>::mux_oh_ao(pop_ptr, mem);
  };
  
  inline void push_no_count_incr (T &push_val) {
    NVHLS_ASSERT_MSG(!full(), "Pushing on FULL!")
    #pragma hls_unroll yes
    for (int i=0; i<SIZE; ++i) {
      bool enable = (push_ptr>>i) & 1;
      if (enable && !full()) mem[i] = push_val;
    }
    inc_push_ptr();
  }
  
  inline void push(T &push_val) {
    NVHLS_ASSERT_MSG(!full(), "Pushing on FULL!")
    #pragma hls_unroll yes
    for (int i=0; i<SIZE; ++i) {
      bool enable = (push_ptr>>i) & 1;
      if (enable && !full()) mem[i] = push_val;
    }
    //mem[push_ptr] = push_val;
    //switch (push_ptr) {
    //  case 1 : mem[0] = push_val;
    //    break;
    //  case 2 : mem[1] = push_val;
    //    break;
    //  case 4 : mem[2] = push_val;
    //    break;
    //  default : ;
    //    break;
    //}

    inc_push_ptr();
    incr_count();
  };
  
  inline T pop() {
    NVHLS_ASSERT_MSG(!empty(), "Popping on EMPTY!")
    T mule = mux<T, SIZE>::mux_oh_case(pop_ptr, mem);
    inc_pop_ptr();
    decr_count();
    return mule;
  };
  
  inline void try_push(bool pushed, T &push_val) {
    #pragma hls_unroll yes
    for (int i = 0; i < SIZE; ++i) {
      bool enable = ((push_ptr >> i) & 1) && pushed; // ToDo : for some reason catapult uses rshift mod/func instead of statically select the bit.
      if (enable) mem[i] = push_val;
    }
    if (pushed) inc_push_ptr();
  };
  
  inline void set_count(bool pushed, bool popped) {
    if      ( pushed && !popped) item_count  = (item_count << 1);
    else if (!pushed &&  popped) item_count  = (item_count >> 1);
  }
  
  inline void inc_pop_ptr()  { pop_ptr  = (pop_ptr <<1) | ((pop_ptr >>(SIZE-1))&1);};    // maybe (pop_ptr<<1)  | pop_ptr[SIZE];  would work equally, although not tested
  inline void inc_push_ptr() { push_ptr = (push_ptr<<1) | ((push_ptr>>(SIZE-1))&1);}; // maybe (push_ptr<<1) | push_ptr[SIZE]; would work equally, although not tested
  
  inline void incr_count() {item_count  = (item_count << 1);};
  inline void decr_count() {item_count  = (item_count >> 1);};

};

#endif // #define __FIFO_QUEUE__
