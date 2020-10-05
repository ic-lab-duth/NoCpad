#ifndef __TB_WRAP_H__
#define __TB_WRAP_H__

#include "systemc.h"
#include "nvhls_connections.h"

#ifndef __SYNTHESIS__
	#include <string>
	#include <iostream>
#endif


template<class T>
struct msg_tb_wrap {
  T dut_msg;
  
  sc_time time_gen;
  sc_time time_inj;
  
  sc_time time_ej;
  
  bool is_read = false;
  
	inline friend std::ostream& operator << ( std::ostream& os, const msg_tb_wrap& msg_tmp ) {
		os << msg_tmp.dut_msg;
		return os;
	}

  // Only for SystemC
  inline friend void sc_trace(sc_trace_file* tf, const msg_tb_wrap& msg, const std::string& name) {
    sc_trace(tf, msg.dut_msg, name);
	}
  
};

#endif // __TB_WRAP_H__

