#ifndef AXI_SLAVE_H
#define AXI_SLAVE_H

#include "systemc.h"

#include "../../src/include/flit_axi.h"
#include "../tb_wrap.h"

#include <deque>
#include <queue>

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>


#define AXI_TID_NUM 5
#define AXI_ADDR_MAX 0xffffffff

template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int MASTER_NUM, unsigned int SLAVE_NUM>
SC_MODULE(axi_slave) {
  typedef typename axi::axi4<axi::cfg::standard_duth> axi4_;
  typedef typename axi::AXI4_Encoding            enc_;
  
	sc_in_clk    clk;
  sc_in <bool> rst_n;
  
	sc_in<bool>  stop_gen; // Not Used
  
  sc_in< sc_uint<32> >  addr_map[SLAVE_NUM][2];
  
	Connections::In<axi4_::AddrPayload>    ar_in;
	Connections::Out<axi4_::ReadPayload>   r_out;
  
  Connections::In<axi4_::AddrPayload>    aw_in;
	Connections::In<axi4_::WritePayload>   w_in;
	Connections::Out<axi4_::WRespPayload>  b_out;
  
  // Scoreboard
  sc_mutex                                                       *sb_lock;
  std::vector< std::deque< msg_tb_wrap<axi4_::AddrPayload> > >   *sb_rd_req_q;
  std::vector< std::deque< msg_tb_wrap<axi4_::ReadPayload> > >   *sb_rd_resp_q; // This is Sized to Master LANES
  
  std::vector< std::deque< msg_tb_wrap<axi4_::AddrPayload> > >   *sb_wr_req_q;
  std::vector< std::deque< msg_tb_wrap<axi4_::WritePayload> > >  *sb_wr_data_q;
  std::vector< std::deque< msg_tb_wrap<axi4_::WRespPayload> > >  *sb_wr_resp_q;
  
	int SLAVE_ID   = -1;
	unsigned int STALL_RATE_RD;
  unsigned int STALL_RATE_WR;
  // int FLOW_CTRL;     // 0: READY-VALID
  //                    // 1: CREDITS 
  //                    // 2: FIFO
  //                    // 3: Credits fifo based
  
	unsigned int total_cycles;
  
  sc_time clk_period;
	
  std::queue<axi4_::ReadPayload >  stored_rd_resp;
  std::queue<axi4_::WRespPayload>  stored_wr_resp;
  
  std::deque<axi4_::AddrPayload>   wr_to_get_resp;
  
  // Read Response Generator
  int rd_resp_val;
  int rd_resp_generated;
  int rd_resp_inj;
  
  int wr_resp_generated;
  int wr_resp_inj;
  
  // Read Req Sink
  int rd_req_ej;
  
  int wr_req_ej;
  int wr_data_ej;
  
  // Error
  int error_sb_rd_req_not_found;
  int error_sb_wr_req_not_found;
  int error_sb_wr_data_not_found;
  
  // Functions
	void do_cycle();
	void gen_rd_resp(axi4_::AddrPayload   &rcv_rd_req );
	void gen_wr_resp(unsigned wr_initiator);
  
	bool verify_rd_req(axi4_::AddrPayload    &rcv_rd_req);
	bool verify_wr_req(axi4_::AddrPayload    &rcv_wr_req);
  bool verify_wr_data(axi4_::WritePayload  &rcv_wr_data, unsigned &wr_initiator);
	
  
  bool eq_rd_req (axi4_::AddrPayload &rcv_rd_req  , axi4_::AddrPayload &sb_rd_req);
  bool eq_wr_req (axi4_::AddrPayload &rcv_wr_req  , axi4_::AddrPayload &sb_wr_req);
  bool eq_wr_data(axi4_::WritePayload &rcv_wr_data, axi4_::WritePayload &sb_wr_data);
  
  // Constructor
  SC_HAS_PROCESS(axi_slave);
  axi_slave(sc_module_name name_) : sc_module(name_)
  {
    SLAVE_ID      = -1;
    STALL_RATE_RD = 0;
    STALL_RATE_WR = 0;
    
		SC_THREAD(do_cycle);
    sensitive << clk.pos();
    async_reset_signal_is(rst_n, false);
  }
};


// --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- //
// --- --- --- --- --- --- IMPLEMENTATION  --- --- --- --- --- --- --- --- //
// --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- //



#include "axi_slave.h"

template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int MASTER_NUM, unsigned int SLAVE_NUM>
void axi_slave<RD_M_LANES, RD_S_LANES, WR_M_LANES, WR_S_LANES, MASTER_NUM, SLAVE_NUM>::do_cycle () {
  total_cycles      = 0;
  
  rd_resp_generated = 0;
  rd_resp_inj       = 0;
  rd_req_ej         = 0;
  
  wr_resp_generated = 0;
  wr_resp_inj       = 0;
  
  wr_req_ej         = 0;
  wr_data_ej        = 0;
  
  rd_resp_val       = 1;
  
  clk_period = (dynamic_cast<sc_clock *>(clk.get_interface()))->period();
  
  // Error
  error_sb_rd_req_not_found  = 0;
  error_sb_wr_req_not_found  = 0;
  error_sb_wr_data_not_found = 0;
  
  while(1) {
    wait();
    // READ REQUESTS
    // Sink/Verify Read Request + Create the appropriate response
    bool do_sink = (rand()%100) >= STALL_RATE_RD;
    if (do_sink) {
      axi4_::AddrPayload rcv_rd_req;
      if (ar_in.PopNB(rcv_rd_req)) {
        sc_time this_gen_time;
        verify_rd_req(rcv_rd_req);
        rd_req_ej++;
  
        gen_rd_resp(rcv_rd_req);
      }
    }
    
    // WRITE REQUESTS
    do_sink = (rand()%100) >= STALL_RATE_RD;
    if (do_sink) {
      axi4_::AddrPayload rcv_wr_req;
      if (aw_in.PopNB(rcv_wr_req)) {
        verify_wr_req(rcv_wr_req);
        wr_to_get_resp.push_back(rcv_wr_req);
        wr_req_ej++;
      }
    }
  
    do_sink = (rand()%100) >= STALL_RATE_WR;
    if (do_sink) {
      axi4_::WritePayload rcv_wr_data;
      if (w_in.PopNB(rcv_wr_data)) {
        unsigned wr_initiator = -1;
        verify_wr_data(rcv_wr_data, wr_initiator);
        wr_data_ej++;
  
        if (rcv_wr_data.last) gen_wr_resp(wr_initiator);
      }
    }
    
    // RESPONSES
    // Inject READ Responses
    bool do_push = (rand()%100) >= STALL_RATE_RD;
    if (do_push && !stored_rd_resp.empty()) {
      axi4_::ReadPayload temp_resp = stored_rd_resp.front();
      if (r_out.PushNB(temp_resp)) {
        stored_rd_resp.pop();
        std::cout<<"[Slave "<< SLAVE_ID << "] : PUSHED RD-Resp " << temp_resp << " @" << sc_time_stamp() << std::endl;
        rd_resp_inj++;
      }
    }
    
    // Inject WRITE Responses
    do_push = (rand()%100) >= STALL_RATE_WR;
    if (do_push && !stored_wr_resp.empty()) {
      axi4_::WRespPayload temp_resp = stored_wr_resp.front();
      if (b_out.PushNB(temp_resp)) {
        stored_wr_resp.pop();
        std::cout<<"[Slave "<< SLAVE_ID << "] : PUSHED WR-Resp " << temp_resp << " @" << sc_time_stamp() << std::endl;
        wr_resp_inj++;
      }
    }
    
  } // End of while(1)
}; // End of do_cycle





// --------------------------- //
// --- GENERATOR Functions --- //
// --------------------------- //
template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int MASTER_NUM, unsigned int SLAVE_NUM>
void axi_slave<RD_M_LANES, RD_S_LANES, WR_M_LANES, WR_S_LANES, MASTER_NUM, SLAVE_NUM>::gen_rd_resp(axi4_::AddrPayload &rcv_rd_req) {
  sb_lock->lock();
  axi4_::ReadPayload  cur_beat;  // Sized for Slave
  axi4_::ReadPayload  beat_at_master; // Sized for master
  
  cur_beat.id       = rcv_rd_req.id;
  beat_at_master.id = rcv_rd_req.id;
  
  // Create Response
  unsigned long int bytes_total = ((rcv_rd_req.len+1)<<rcv_rd_req.size);
  unsigned long int byte_count  = 0;
  
  unsigned char s_init_ptr = rcv_rd_req.addr % RD_S_LANES;
  unsigned char s_ptr      = s_init_ptr;
  unsigned char s_size     = rcv_rd_req.size;
  
  cur_beat.data = 0;
  beat_at_master.data = 0;
  
  while(byte_count<bytes_total) {
    cur_beat.data |= ( ((axi4_::Data)(byte_count & 0xFF)) << ((axi4_::Data)(s_ptr*8)));
    byte_count++;
    
    s_ptr = (rcv_rd_req.burst==enc_::AXBURST::FIXED) ? ((s_ptr+1)%(1<<s_size)) + s_init_ptr
                                                     :  (s_ptr+1)%RD_S_LANES ;
    
    if(((s_ptr%(1<<s_size))==0) || (byte_count == bytes_total)) {
      cur_beat.resp = SLAVE_ID;
      cur_beat.last = (byte_count == bytes_total);
      stored_rd_resp.push(cur_beat);
      
      cur_beat.data = 0;
    }
  }
  sb_lock->unlock();
}; // End of Read generator

template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int MASTER_NUM, unsigned int SLAVE_NUM>
void axi_slave<RD_M_LANES, RD_S_LANES, WR_M_LANES, WR_S_LANES, MASTER_NUM, SLAVE_NUM>::gen_wr_resp(unsigned wr_initiator) {
  sb_lock->lock();
  axi4_::AddrPayload rcv_wr_req = wr_to_get_resp.front();
  wr_to_get_resp.pop_front();
  
  // Create Response
  axi4_::WRespPayload temp_wr_resp;
  temp_wr_resp.id   = rcv_wr_req.id;
  temp_wr_resp.resp = SLAVE_ID;
  
  stored_wr_resp.push(temp_wr_resp); // Send WR-Resp to DUT
  
  msg_tb_wrap<axi4_::WRespPayload> temp_wr_resp_tb;
  temp_wr_resp_tb.dut_msg  = temp_wr_resp;
  temp_wr_resp_tb.time_gen = sc_time_stamp();
  
  (*sb_wr_resp_q)[wr_initiator].push_back(temp_wr_resp_tb); // Send Beat to ScoreBoard.
  wr_resp_generated++;
  sb_lock->unlock();
}; // End of Read generator




// ------------------------ //
// --- VERIFY Functions --- //
// ------------------------ //
template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int MASTER_NUM, unsigned int SLAVE_NUM>
bool axi_slave<RD_M_LANES, RD_S_LANES, WR_M_LANES, WR_S_LANES, MASTER_NUM, SLAVE_NUM>::verify_rd_req (axi4_::AddrPayload &rcv_rd_req) {
  bool verified = true;
  sb_lock->lock();
  bool found=false;
  unsigned int  j=0;
  while (j<(*sb_rd_req_q)[SLAVE_ID].size()){
    msg_tb_wrap< axi4_::AddrPayload > sb_req = (*sb_rd_req_q)[SLAVE_ID][j];
    
    if ( eq_rd_req(rcv_rd_req, sb_req.dut_msg) ){
      (*sb_rd_req_q)[SLAVE_ID].erase((*sb_rd_req_q)[SLAVE_ID].begin()+j);
      found = true;
      break;
    }
    j++;
  }
  
  if(!found){
    std::cout << "ERR : [Slave " << SLAVE_ID <<"] " << "RD Request   : "<< rcv_rd_req << " . NOT FOUND! @" << sc_time_stamp() << "\n";
    std::cout << "ERR : [Slave " << SLAVE_ID <<"] " << "-SB_front - "<< (*sb_rd_req_q)[SLAVE_ID].front() << "\n";
    error_sb_rd_req_not_found++;
    sc_assert(0);
    // sc_stop();
    verified = false;
  } else {
    std::cout<< "[Slave " << SLAVE_ID <<"] " << "RD Req OK  : <<  " << rcv_rd_req << " @" << sc_time_stamp() << "\n";
  }
  std::cout.flush();
  sb_lock->unlock();
  return verified;
}; // End of READ Req Verify

template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int MASTER_NUM, unsigned int SLAVE_NUM>
bool axi_slave<RD_M_LANES, RD_S_LANES, WR_M_LANES, WR_S_LANES, MASTER_NUM, SLAVE_NUM>::verify_wr_req (axi4_::AddrPayload &rcv_wr_req) {
  bool verified = true;
  sb_lock->lock();
  bool found=false;
  unsigned int  j=0;
  while (j<(*sb_wr_req_q)[SLAVE_ID].size()){
    axi4_::AddrPayload sb_value = ((*sb_wr_req_q)[SLAVE_ID][j]).dut_msg;
    
    if (eq_wr_req(rcv_wr_req, sb_value)){
      (*sb_wr_req_q)[SLAVE_ID].erase((*sb_wr_req_q)[SLAVE_ID].begin()+j);
      found = true;
      break;
    }
    j++;
  }
  
  if(!found){
    std::cout << "\n";
    std::cout << "ERR : [Slave " << SLAVE_ID <<"] " << "WR Request   : "<< rcv_wr_req << " . NOT FOUND! @" << sc_time_stamp() << "\n";
    std::cout << "ERR : [Slave " << SLAVE_ID <<"] " << "-SB_front - "<< (*sb_wr_req_q)[SLAVE_ID].front() << "\n";
    error_sb_wr_req_not_found++;
    sc_assert(0);
    // sc_stop();
    verified = false;
  } else {
    std::cout<< "[Slave " << SLAVE_ID <<"] " << "WR Req OK  : <<  " << rcv_wr_req << "\n";
  }
  std::cout.flush();
  sb_lock->unlock();
  return verified;
}; // End of WRITE Req Verify

template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int MASTER_NUM, unsigned int SLAVE_NUM>
bool axi_slave<RD_M_LANES, RD_S_LANES, WR_M_LANES, WR_S_LANES, MASTER_NUM, SLAVE_NUM>::verify_wr_data (axi4_::WritePayload &rcv_wr_data, unsigned &wr_initiator) {
  bool verified = true;
  sb_lock->lock();
  bool found=false;
  unsigned int  j=0;
  while (j<(*sb_wr_data_q)[SLAVE_ID].size()){
    axi4_::WritePayload sb_value = ((*sb_wr_data_q)[SLAVE_ID][j]).dut_msg;
    
    if (eq_wr_data(rcv_wr_data, sb_value)){
      // The last byte of the last beat signals the initiator
      if (rcv_wr_data.last.to_uint()) {
        for (int i=WR_S_LANES-1; i>=0;--i) {
          if ( (rcv_wr_data.wstrb.to_uint() >> i) & 1 ) {
            wr_initiator = ((rcv_wr_data.data >> (i*8)) & 0xFF).to_uint();
            break;
          }
        }
      }
      
      (*sb_wr_data_q)[SLAVE_ID].erase((*sb_wr_data_q)[SLAVE_ID].begin()+j);
      found = true;
      break;
    }
    j++;
  }
  
  if(!found){
    std::cout << "ERR : [Slave " << SLAVE_ID <<"] " << "WR Data   : "<< rcv_wr_data << " . NOT FOUND! @" << sc_time_stamp() << "\n";
    std::cout << "ERR : [Slave " << SLAVE_ID <<"] " << "-SB_front - "<< (*sb_wr_data_q)[SLAVE_ID].front() << "\n";
    error_sb_wr_data_not_found++;
    sc_assert(0);
    // sc_stop();
    verified = false;
  } else {
    std::cout<< "[Slave " << SLAVE_ID <<"] " << "WR Data OK  : <<  " << rcv_wr_data << "\n";
  }
  std::cout.flush();
  sb_lock->unlock();
  return verified;
}; // End of WRITE Data Verify


template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int MASTER_NUM, unsigned int SLAVE_NUM>
bool axi_slave<RD_M_LANES, RD_S_LANES, WR_M_LANES, WR_S_LANES, MASTER_NUM, SLAVE_NUM>::eq_rd_req (axi4_::AddrPayload &rcv_rd_req, axi4_::AddrPayload &sb_rd_req) {
  bool equal = true;
  unsigned tid_mask = (1<<dnp::ID_W)-1;
  equal = equal && ((rcv_rd_req.id & tid_mask) == (sb_rd_req.id & tid_mask));
  equal = equal && (rcv_rd_req.addr  == (sb_rd_req.addr - addr_map[SLAVE_ID][0].read()));
  equal = equal && (rcv_rd_req.burst == sb_rd_req.burst);
  equal = equal && (rcv_rd_req.len   == sb_rd_req.len);
  equal = equal && (rcv_rd_req.size  == sb_rd_req.size);
  //equal = equal && (rcv_rd_req.cache == sb_rd_req.cache);
  //equal = equal && (rcv_rd_req.auser == sb_rd_req.auser);
  //equal = equal && (rcv_rd_req.snoop   == sb_rd_req.snoop);
  //equal = equal && (rcv_rd_req.domain  == sb_rd_req.domain);
  //equal = equal && (rcv_rd_req.barrier == sb_rd_req.barrier);
  return equal;
};

template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int MASTER_NUM, unsigned int SLAVE_NUM>
bool axi_slave<RD_M_LANES, RD_S_LANES, WR_M_LANES, WR_S_LANES, MASTER_NUM, SLAVE_NUM>::eq_wr_req (axi4_::AddrPayload &rcv_wr_req, axi4_::AddrPayload &sb_wr_req) {
  bool equal = true;
  unsigned tid_mask = (1<<dnp::ID_W)-1;
  equal = equal && ((rcv_wr_req.id & tid_mask) == (sb_wr_req.id & tid_mask));
  equal = equal && (rcv_wr_req.addr  == (sb_wr_req.addr - addr_map[SLAVE_ID][0].read()));
  equal = equal && (rcv_wr_req.burst == sb_wr_req.burst);
  equal = equal && (rcv_wr_req.len   == sb_wr_req.len);
  equal = equal && (rcv_wr_req.size  == sb_wr_req.size);
  //equal = equal && (rcv_wr_req.cache == sb_wr_req.cache);
  //equal = equal && (rcv_wr_req.auser == sb_wr_req.auser);
  //equal = equal && (rcv_wr_req.snoop   == sb_wr_req.snoop);
  //equal = equal && (rcv_wr_req.domain  == sb_wr_req.domain);
  //equal = equal && (rcv_wr_req.barrier == sb_wr_req.barrier);
  return equal;
};

template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int MASTER_NUM, unsigned int SLAVE_NUM>
bool axi_slave<RD_M_LANES, RD_S_LANES, WR_M_LANES, WR_S_LANES, MASTER_NUM, SLAVE_NUM>::eq_wr_data (axi4_::WritePayload &rcv_wr_data, axi4_::WritePayload &sb_wr_data) {
  bool equal = true;
  unsigned tid_mask = (1<<dnp::ID_W)-1;
  for(int i=0; i<WR_S_LANES; ++i) {
    equal = equal && (((rcv_wr_data.wstrb >> i) & 1) == ((sb_wr_data.wstrb >> i) & 1));
    if ((rcv_wr_data.wstrb >> i) & 1) {
      equal = equal && (((rcv_wr_data.data >> (8 * i)) & 0xFF) == ((sb_wr_data.data >> (8 * i)) & 0xFF));
    }
  }
  return (equal &&
          rcv_wr_data.last  == sb_wr_data.last //  &&
          //rcv_wr_data.wuser == sb_wr_data.wuser
  );
};


#endif // AXI_SLAVE_H
