#ifndef _ACE_MASTER_H_
#define _ACE_MASTER_H_

#include "systemc.h"

#include "../helper_non_synth.h"
#include "../../src/include/dnp_ace_v0.h"
#include "../tb_wrap.h"

#include <deque>
#include <queue>

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

#define AXI4_MAX_LEN      4     // FIXED, WRAP bursts has a maximum of 16 beats
#define AXI4_MAX_INCR_LEN 4    // AXI4 extends INCR bursts upto 256 beats

#define AXI_TID_NUM 4
#define AXI_BURST_NUM 3

#define ACE_CACHE_LINES 8


template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int MASTER_NUM, unsigned int SLAVE_NUM>
SC_MODULE(ace_master) {
  typedef typename ace::ace5<axi::cfg::ace> ace5_;
  typedef typename ace::ACE_Encoding        enc_;
  
	sc_in_clk    clk;
  sc_in <bool> rst_n;

	sc_in<bool>  stop_gen;
 
	sc_in< sc_uint<32> >  addr_map[SLAVE_NUM][2];
  
  // Master ACE Ports
  Connections::In<ace5_::AC>             ac_in;
  Connections::Out<ace5_::CR>            cr_out;
  Connections::Out<ace5_::CD>            cd_out;
  
  Connections::Out<ace5_::AddrPayload>   ar_out;
  Connections::In<ace5_::ReadPayload>    r_in;
  Connections::Out<ace5_::RACK>          rack_out;
  
  Connections::Out<ace5_::AddrPayload>   aw_out;
  Connections::Out<ace5_::WritePayload>  w_out;
  Connections::In<ace5_::WRespPayload>   b_in;
  Connections::Out<ace5_::WACK>          wack_out;
  
  // Scoreboard
  sc_mutex                                                      *sb_lock;
  std::vector< std::deque< msg_tb_wrap<ace5_::AddrPayload> > >  *sb_rd_req_q;
  std::vector< std::deque< msg_tb_wrap<ace5_::ReadPayload> > >  *sb_rd_resp_q;
  
  std::vector< std::deque< msg_tb_wrap<ace5_::AddrPayload > > >  *sb_wr_req_q;
  std::vector< std::deque< msg_tb_wrap<ace5_::WritePayload> > >  *sb_wr_data_q;
  std::vector< std::deque< msg_tb_wrap<ace5_::WRespPayload> > >  *sb_wr_resp_q;
  
  std::vector< std::deque< msg_tb_wrap<ace5_::AddrPayload> > >   *sb_coherent_access_q;
  std::vector< std::deque< msg_tb_wrap<ace5_::AC> > >            *sb_snoop_req_q;
  std::vector< std::deque< msg_tb_wrap<ace5_::CR> > >            *sb_snoop_resp_q;
  std::vector< std::deque< msg_tb_wrap<ace5_::CD> > >            *sb_snoop_data_resp_q;
  
  // Queues to store generated transactions
  std::queue<ace5_::AddrPayload >  stored_rd_trans;
  std::queue<ace5_::AddrPayload >  stored_wr_trans;
  std::queue<ace5_::WritePayload>  stored_wr_data;
  
  std::queue<ace5_::CR>  stored_cache_resp;
  std::queue<ace5_::CD>  stored_cache_data;
  
  std::queue<ace5_::RACK>  stored_rd_ack;
  std::queue<ace5_::WACK>  stored_wr_ack;
  
  std::deque<ace5_::AddrPayload>  sb_rd_order_q; // queue to check ordering
  std::deque<ace5_::AddrPayload>  sb_wr_order_q; // queue to check ordering
  
  // Cache state keeping
  class cache_line {
  public:
      enum State {
        INV = 0, // INVALID
        UC  = 1, // UNIQUE_CLEAN
        UD  = 2, // UNIQUE_DIRTY
        SC  = 3, // SHARED_CLEAN
        SD  = 4, // SHARED_DIRTY
      };
      
      ace5_::CD::Data data;
      State state;
      
      cache_line () {
        data  = 0;
        state = INV;
      }
      cache_line (ace5_::CD::Data data_, State state_) {
        data  = data_;
        state = state_;
      }
      
      inline bool is_inv() {return (state == INV);};
      inline bool is_uc()  {return (state == UC);};
      inline bool is_ud()  {return (state == UD);};
      inline bool is_sc()  {return (state == SC);};
      inline bool is_sd()  {return (state == SD);};
      
      inline friend std::ostream& operator<<(ostream& os, const cache_line& rhs)
      {
        os << std::hex;
        os << "Data:" << rhs.data << " ";
        os << std::dec;
        if     (rhs.state == INV) os << "State: I";
        else if(rhs.state == UC)  os << "State: UC";
        else if(rhs.state == UD)  os << "State: UD";
        else if(rhs.state == SC)  os << "State: SC";
        else if(rhs.state == SD)  os << "State: SD";
        else                      os << "State: ??";
        
        return os;
      }
  };
  
  std::map<ace5_::Addr, cache_line> cache;
  std::map<ace5_::Addr, int>        cache_outstanding;
  std::map<ace5_::Addr, int>        cache_outstanding_writes;
  
	int MASTER_ID  = -1;
	unsigned int AXI_GEN_RATE_RD;
  unsigned int AXI_GEN_RATE_WR;
  unsigned int ACE_GEN_RATE_CACHE;
  // int FLOW_CTRL;     // 0: READY-VALID
  //                    // 1: CREDITS 
  //                    // 2: FIFO
  //                    // 3: Credits fifo based
  
  // delays
	long     total_cycles;
  sc_time clk_period;
  
  unsigned long long int rd_resp_delay = 0;
  unsigned long long int rd_resp_count = 0;
  unsigned long long int wr_resp_delay = 0;
  unsigned long long int wr_resp_count = 0;
  unsigned long long int last_rd_sinked_cycle = 0;
  unsigned long long int last_wr_sinked_cycle = 0;
  unsigned long long int rd_resp_data_count = 0;
  unsigned long long int wr_resp_data_count = 0;
  
	bool stop_at_tail, has_stopped_gen;
	

  
  // Read Addr Generator
  int cache_trans_generated;
  int rd_trans_generated;
  int rd_data_generated;
  int wr_trans_generated;
  int wr_data_generated;
  
  int rd_trans_inj;
  int wr_trans_inj;
  int wr_data_inj;
  
  unsigned int gen_rd_addr;
  unsigned int gen_wr_addr;
  unsigned int resp_val_expect;
  
  // Read Responce Sink
  int rd_resp_ej;
  int wr_resp_ej;
  
  // Errors
  int error_sb_rd_resp_not_found;
  int error_sb_wr_resp_not_found;
  
  // Functions
	void do_cycle();
	void gen_new_rd_trans();
	void gen_new_wr_trans();
	void gen_new_cache_trans();
	
	void gen_snoop_resp(ace5_::AC &rcv_snoop_req);
	
	void upd_cache_read(ace5_::AddrPayload req, ace5_::ReadPayload);
	void upd_cache_write(ace5_::AddrPayload req, ace5_::WRespPayload);
 
	void verify_rd_resp(ace5_::ReadPayload  &rcv_rd_resp);
	void verify_wr_resp(ace5_::WRespPayload &rcv_wr_resp);
  
  bool eq_rd_data(ace5_::ReadPayload &rcv_rd_data, ace5_::ReadPayload &sb_rd_data);
  bool eq_wr_resp(ace5_::WRespPayload &rcv_wr_resp, ace5_::WRespPayload &sb_wr_resp);
	
	unsigned mem_map_resolve(ace5_::Addr &addr);
  
  // Constructor
  SC_HAS_PROCESS(ace_master);
  ace_master(sc_module_name name_) : sc_module(name_)
  {
    MASTER_ID = -1;
    AXI_GEN_RATE_RD      = 0;
    AXI_GEN_RATE_WR      = 0;
    ACE_GEN_RATE_CACHE   = 5;
    
		SC_THREAD(do_cycle);
    sensitive << clk.pos();
    reset_signal_is(rst_n, false);
  }
};


// --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- //
// --- --- --- --- --- --- IMPLEMENTATION  --- --- --- --- --- --- --- --- //
// --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- //

template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int MASTER_NUM, unsigned int SLAVE_NUM>
void ace_master<RD_M_LANES, RD_S_LANES, WR_M_LANES, WR_S_LANES, MASTER_NUM, SLAVE_NUM>::do_cycle () {
  total_cycles       = 0;
  
  cache_trans_generated = 0;
  rd_trans_generated = 0;
  rd_data_generated  = 0;
  wr_trans_generated = 0;
  wr_data_generated  = 0;
  rd_trans_inj       = 0;
  wr_trans_inj       = 0;
  wr_data_inj        = 0;
  gen_rd_addr        = 0x10100;
  gen_wr_addr        = 0;
  resp_val_expect    = 0;
  
  rd_resp_ej = 0;
  wr_resp_ej = 0;
  
  // Clear errors
  error_sb_rd_resp_not_found = 0;
  error_sb_wr_resp_not_found = 0;
  
  clk_period = (dynamic_cast<sc_clock *>(clk.get_interface()))->period();
  
  ac_in.Reset();
  cr_out.Reset();
  cd_out.Reset();
  
  ar_out.Reset();
  r_in.Reset();
  rack_out.Reset();
  
  aw_out.Reset();
  w_out.Reset();
  b_in.Reset();
  wack_out.Reset();
  
  //if (MASTER_ID == 2) cache[8] = cache_line(0xFF00FF00FF00FF00, cache_line::State::UC);
  //if (MASTER_ID == 3) cache[8] = cache_line(0xAA00AA00AA00AA00, cache_line::State::SD);
  //if (MASTER_ID == 4) cache[8] = cache_line(0xFF00FF00FF00FF00, cache_line::State::SC);
  
  while(1) {
    wait();
    total_cycles++;
    
    // Transaction Generator
    if (!stop_gen.read()) {
      unsigned int rnd_val_rd = rand()%100;
      if (rnd_val_rd < AXI_GEN_RATE_RD) {
        gen_new_rd_trans();
      }
      
      unsigned int rnd_val_wr = rand()%100;
      if (rnd_val_wr < AXI_GEN_RATE_WR) {
        gen_new_wr_trans();
      }
  
      unsigned int rnd_val_cache = rand()%100;
      if (rnd_val_cache < ACE_GEN_RATE_CACHE) {
        gen_new_cache_trans();
      }
    }
    
    // Read Request Injection
    if (!stored_rd_trans.empty()) {
      ace5_::AddrPayload tmp_ar = stored_rd_trans.front();
      bool is_coherent                 = (tmp_ar.snoop || tmp_ar.domain.xor_reduce());
      int  coherent_writes_outstanding = cache_outstanding_writes[tmp_ar.addr];
      if (!(is_coherent && coherent_writes_outstanding)) {
        if (ar_out.PushNB(tmp_ar)) {
          stored_rd_trans.pop();
    
          std::cout << "[Master " << MASTER_ID << "] : PUSHED AR:" << tmp_ar << " @" << sc_time_stamp() << std::endl;
          rd_trans_inj++;
          if(is_coherent) {
            cache_outstanding[tmp_ar.addr]++;
  
            // Push it to ACE Checker
            sb_lock->lock();
            msg_tb_wrap<ace5_::AddrPayload> temp_rd_coherent_req_tb;
            temp_rd_coherent_req_tb.dut_msg  = tmp_ar;
            temp_rd_coherent_req_tb.is_read  = true;
            temp_rd_coherent_req_tb.time_gen = sc_time_stamp();
            
            (*sb_coherent_access_q)[MASTER_ID - SLAVE_NUM].push_back(temp_rd_coherent_req_tb);
            sb_lock->unlock();
          }
        }
      }

    }
    
    // Write Request Injection
    if (!stored_wr_trans.empty()) {
      ace5_::AddrPayload tmp_aw = stored_wr_trans.front();
      bool is_coherent          = (tmp_aw.snoop || tmp_aw.domain.xor_reduce());
      int  coherent_outstanding = cache_outstanding[tmp_aw.addr];
      if (!(is_coherent && coherent_outstanding)) {
        if (aw_out.PushNB(tmp_aw)) {
          stored_wr_trans.pop();
    
          std::cout << "[Master " << MASTER_ID << "] : PUSHED AW: " << tmp_aw << " @" << sc_time_stamp() << std::endl;
          wr_trans_inj++;
          if(is_coherent) {
            cache_outstanding[tmp_aw.addr]++;
            cache_outstanding_writes[tmp_aw.addr]++;
  
            sb_lock->lock();
            msg_tb_wrap<ace5_::AddrPayload> temp_wr_coherent_req_tb;
            temp_wr_coherent_req_tb.dut_msg  = tmp_aw;
            temp_wr_coherent_req_tb.is_read  = false;
            temp_wr_coherent_req_tb.time_gen = sc_time_stamp();
            
            (*sb_coherent_access_q)[MASTER_ID - SLAVE_NUM].push_back(temp_wr_coherent_req_tb);
            sb_lock->unlock();
          }
        }
      }
    }
    
    // Write Data Injection
    if (!stored_wr_data.empty()) {
      ace5_::WritePayload tmp_w = stored_wr_data.front();
      if (w_out.PushNB(tmp_w)) {
        stored_wr_data.pop();
  
        std::cout << "[Master " << MASTER_ID << "] : PUSHED W: " << tmp_w << " @" << sc_time_stamp() << std::endl;
        wr_data_inj++;
      }
    }
    
    // Read Response Ejection
    ace5_::ReadPayload rcv_rd_resp;
    bool got_ar_resp = r_in.PopNB(rcv_rd_resp);  // Lacks backpressure
    if(got_ar_resp){
      verify_rd_resp(rcv_rd_resp);
    }
    
    // Write Response Ejection
    ace5_::WRespPayload rcv_wr_resp;
    bool got_b_resp = b_in.PopNB(rcv_wr_resp);  // Lacks backpressure
    if(got_b_resp){
      verify_wr_resp(rcv_wr_resp);
    }
    
    
    // --- ACE --- //
    // Snoop Request Ejection
    ace5_::AC rcv_snoop_req;
    bool got_snoop_req = ac_in.PopNB(rcv_snoop_req);  // Lacks backpressure
    if(got_snoop_req){
      // ToDo : Implement ACE verification
      //verify_snoop_req(got_snoop_req);
      gen_snoop_resp(rcv_snoop_req);
    }
  
    // Snoop Response Injection
    if (!stored_cache_resp.empty()) {
      ace5_::CR tmp_cr = stored_cache_resp.front();
      if (cr_out.PushNB(tmp_cr)) {
        stored_cache_resp.pop();
      
        std::cout << "[Master " << MASTER_ID << "] : PUSHED SNOOP Resp: " << tmp_cr << " @" << sc_time_stamp() << std::endl;
        //cache_resp_inj++;
      }
    }
  
    // Snoop Data Response Injection
    if (!stored_cache_data.empty()) {
      ace5_::CD tmp_cd = stored_cache_data.front();
      if (cd_out.PushNB(tmp_cd)) {
        stored_cache_data.pop();
      
        std::cout << "[Master " << MASTER_ID << "] : PUSHED SNOOP Data: " << tmp_cd << " @" << sc_time_stamp() << std::endl;
        //cache_resp_data_inj++;
      }
    }
  
    // READ Ack Injection
    if (!stored_rd_ack.empty()) {
      ace5_::RACK tmp_rack = stored_rd_ack.front();
      if (rack_out.PushNB(tmp_rack)) {
        stored_rd_ack.pop();
        std::cout << "[Master " << MASTER_ID << "] : PUSHED READ Ack: " << tmp_rack << " @" << sc_time_stamp() << std::endl;
      }
    }
  
    // WRITE Ack Injection
    if (!stored_wr_ack.empty()) {
      ace5_::WACK tmp_wack = stored_wr_ack.front();
      if (wack_out.PushNB(tmp_wack)) {
        stored_wr_ack.pop();
        std::cout << "[Master " << MASTER_ID << "] : PUSHED WRITE Ack: " << tmp_wack << " @" << sc_time_stamp() << std::endl;
      }
    }
    
  }; // End of while(1)
}; // End of do_cycle

// --------------------------- //
// --- GENERATOR Functions --- //
// --------------------------- //
template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int MASTER_NUM, unsigned int SLAVE_NUM>
void ace_master<RD_M_LANES, RD_S_LANES, WR_M_LANES, WR_S_LANES, MASTER_NUM, SLAVE_NUM>::gen_new_rd_trans() {
  ace5_::AddrPayload rd_req_m;//(SINGLE, -1, -1, -1);
  
  rd_req_m.id    = (rand()%AXI_TID_NUM); // (rand()% 2)+2;
  rd_req_m.size  = ((rand()%my_log2c(RD_M_LANES))+1) & ((1<<my_log2c(RD_M_LANES))-1);
  rd_req_m.burst = (rand()%AXI_BURST_NUM);
  rd_req_m.len   = (rd_req_m.burst==enc_::AXBURST::WRAP)  ? ((1<<(rand()%my_log2c(AXI4_MAX_LEN+1)))-1)           :
                   (rd_req_m.burst==enc_::AXBURST::FIXED) ? (rand()%AXI4_MAX_LEN)                                :
                   (RD_M_LANES>RD_S_LANES)                ? (rand()%(AXI4_MAX_INCR_LEN/(RD_M_LANES/RD_S_LANES)))    // INCR With    Downsize // Cap the maximum len in case of transactions downsize (which increases len)
                                                          : (rand()%AXI4_MAX_INCR_LEN)                           ;  // INCR WithOut Downsize
  
  // Increasing address to keep track of the transactions
  rd_req_m.addr = gen_rd_addr + ((rand()%RD_M_LANES) & (1<<rd_req_m.size));
  gen_rd_addr   = (gen_rd_addr + RD_M_LANES) % (addr_map[SLAVE_NUM-1][1].read()+1);
  
  // Push it to injection queue
  stored_rd_trans.push(rd_req_m);
  
  // Push it to Scoreboard
  sb_lock->lock();
  // Consider resizing
  ace5_::AddrPayload rd_req_s;
  rd_req_s.id    = rd_req_m.id;
  rd_req_s.size  = ((1<<rd_req_m.size)>RD_S_LANES) ? my_log2c(RD_S_LANES) : rd_req_m.size.to_uint();
  rd_req_s.len   = ((1<<rd_req_m.size)>RD_S_LANES) ? (((rd_req_m.len.to_uint()+1)<<(rd_req_m.size.to_uint()-my_log2c(RD_S_LANES)))-1) : rd_req_m.len.to_uint();
  rd_req_s.burst = rd_req_m.burst;
  rd_req_s.addr  = rd_req_m.addr;
  
  msg_tb_wrap<ace5_::AddrPayload> temp_rd_req_tb;
  temp_rd_req_tb.dut_msg = rd_req_s;
  
  temp_rd_req_tb.time_gen = sc_time_stamp();
  
  unsigned dst = mem_map_resolve(rd_req_s.addr);
  (*sb_rd_req_q)[dst].push_back(temp_rd_req_tb);
  sb_lock->unlock();
  
  // Push into order queue - Reorder check extension
  sb_rd_order_q.push_back(rd_req_m);
  
  rd_trans_generated++;
  
  // --- --- --- --- --- --- --- --- //
  // Generate the expected Responce
  // --- --- --- --- --- --- --- --- //
  sb_lock->lock();
  ace5_::ReadPayload beat_expected;
  
  beat_expected.id = rd_req_m.id;
  
  // Create Expected Response
  unsigned long int bytes_total = ((rd_req_m.len+1)<<rd_req_m.size);
  unsigned long int byte_count  = 0;
  
  unsigned char m_init_ptr  = rd_req_m.addr % RD_M_LANES;
  unsigned char m_ptr       = m_init_ptr;
  unsigned char m_size      = rd_req_m.size;
  
  //beat_expected.reset_data();
  beat_expected.data = 0;
  
  while(byte_count<bytes_total) {
    beat_expected.data |= ( ((ace5_::Data)(byte_count & 0xFF)) << ((ace5_::Data)(m_ptr*8)));
    byte_count++;
  
    m_ptr = (rd_req_m.burst==FIXED) ? ((m_ptr+1)%(1<<m_size)) + m_init_ptr
                                    :  (m_ptr+1)%RD_M_LANES ;
    
    if(((m_ptr%(1<<m_size))==0) || (byte_count == bytes_total)) {
      beat_expected.resp = mem_map_resolve(rd_req_m.addr);
      beat_expected.last = (byte_count == bytes_total);
      
      msg_tb_wrap< ace5_::ReadPayload > temp_rd_resp_tb;
      temp_rd_resp_tb.dut_msg  = beat_expected;
      temp_rd_resp_tb.time_gen = sc_time_stamp();
      
      (*sb_rd_resp_q)[MASTER_ID-SLAVE_NUM].push_back(temp_rd_resp_tb);
      beat_expected.data = 0;
  
      rd_data_generated++;
    }
  }
  sb_lock->unlock();
}; // End of Read generator

template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int MASTER_NUM, unsigned int SLAVE_NUM>
void ace_master<RD_M_LANES, RD_S_LANES, WR_M_LANES, WR_S_LANES, MASTER_NUM, SLAVE_NUM>::gen_new_wr_trans() {
  sb_lock->lock();
  ace5_::AddrPayload m_wr_req;//(SINGLE, -1, -1, -1);
  
  m_wr_req.id    = (rand()%AXI_TID_NUM) | (MASTER_ID << 2); // (rand()%4)+2;
  m_wr_req.size  = ((rand()%my_log2c(WR_M_LANES))+1) & ((1<<my_log2c(WR_M_LANES))-1); // 0 size is NOT supported
  m_wr_req.burst = (rand()%AXI_BURST_NUM);
  m_wr_req.len   = (m_wr_req.burst==enc_::AXBURST::WRAP)  ? ((1<<(rand()%my_log2c(AXI4_MAX_LEN+1)))-1)           :
                   (m_wr_req.burst==enc_::AXBURST::FIXED) ? (rand()%AXI4_MAX_LEN)                                :
                   (WR_M_LANES>WR_S_LANES) ? (rand()%(AXI4_MAX_INCR_LEN/(WR_M_LANES/WR_S_LANES)))    // INCR With    Downsize // Cap the maximum len in case of transactions downsize (which increases len)
                                           : (rand()%AXI4_MAX_INCR_LEN)                           ;  // INCR WithOut Downsize
  
  // Increasing address to keep track of the transactions
  // Aligned on size transactions (Although non-aligned should be an easy addition)
  m_wr_req.addr  = gen_wr_addr + ((rand()%WR_M_LANES) & (1<<m_wr_req.size));
  gen_wr_addr    = (gen_wr_addr + WR_M_LANES) % (addr_map[SLAVE_NUM-1][1].read()+1);;
  
  // Push it to injection queue
  stored_wr_trans.push(m_wr_req);
  
  // Push into order queue - Reorder check extension
  sb_wr_order_q.push_back(m_wr_req);
  
  // Create dummy write data
  ace5_::WritePayload cur_beat;      // The beat that will be injected at MASTER
  ace5_::WritePayload beat_at_slave; // The expected beat ejected at SLAVE
  
  unsigned long int bytes_total = ((m_wr_req.len+1)<<m_wr_req.size);
  unsigned long int byte_count  = 0;
  
  unsigned char m_init_ptr = m_wr_req.addr % WR_M_LANES;
  unsigned char s_init_ptr = m_wr_req.addr % WR_S_LANES;
  
  unsigned char m_ptr = m_init_ptr;
  unsigned char s_ptr = s_init_ptr;
  
  unsigned char m_size = m_wr_req.size;
  unsigned char s_size = ((1<<m_size)>WR_S_LANES) ? my_log2c(WR_S_LANES) : m_size;

  unsigned char m_len = m_wr_req.len;
  unsigned char s_len = ((1<<m_size)>WR_S_LANES) ? (((m_len+1)<<(m_size-s_size))-1) : m_len;
  
  // Push it to Scoreboard
  ace5_::AddrPayload s_wr_req;
  s_wr_req.id    = m_wr_req.id;
  s_wr_req.addr  = m_wr_req.addr;
  s_wr_req.size  = s_size;
  s_wr_req.len   = s_len;
  s_wr_req.burst = m_wr_req.burst;
  
  msg_tb_wrap<ace5_::AddrPayload> temp_wr_req_tb;
  temp_wr_req_tb.dut_msg = s_wr_req;
  
  unsigned dst = mem_map_resolve(s_wr_req.addr);
  (*sb_wr_req_q)[dst].push_back(temp_wr_req_tb);
  
  cur_beat.data  = 0;
  cur_beat.wstrb = 0;
  beat_at_slave.data  = 0;
  beat_at_slave.wstrb = 0;
  
  while(byte_count<bytes_total) {
    unsigned byte_to_write = (byte_count==bytes_total-1) ? MASTER_ID : byte_count;
    cur_beat.data |= (((ace5_::Data)(byte_to_write & 0xFF)) << ((ace5_::Data)(m_ptr*8)));
    cur_beat.wstrb |= (((ace5_::Data)1) << ((ace5_::Data)m_ptr));
    
    beat_at_slave.data  |= (((ace5_::Data)(byte_to_write & 0xFF)) << ((ace5_::Data)(s_ptr*8)));
    beat_at_slave.wstrb |= (((ace5_::Data)1) << ((ace5_::Data)s_ptr));
    
    byte_count++;
  
    m_ptr = (m_wr_req.burst==FIXED) ? ((m_ptr+1)%(1<<m_size)) + m_init_ptr :
                                      (m_ptr+1)%WR_M_LANES ;
    
    s_ptr = (s_wr_req.burst==FIXED) ? ((s_ptr+1)%(1<<s_size)) + s_init_ptr :
                                      (s_ptr+1)%WR_S_LANES ;
    
    if(((m_ptr%(1<<m_size))==0) || (byte_count == bytes_total)) {
      wr_data_generated++;
      cur_beat.last = (byte_count == bytes_total);
      stored_wr_data.push(cur_beat);
      cur_beat.data  = 0;
      cur_beat.wstrb = 0;
    }

    if(((s_ptr%(1<<s_size))==0) || (byte_count == bytes_total)) {
      beat_at_slave.last = (byte_count == bytes_total);
      msg_tb_wrap< ace5_::WritePayload > temp_wr_data_tb;
      temp_wr_data_tb.dut_msg = beat_at_slave;
        
      wr_resp_data_count++;
      last_wr_sinked_cycle = (sc_time_stamp() / clk_period);
      
      (*sb_wr_data_q)[dst].push_back(temp_wr_data_tb);
      beat_at_slave.data  = 0;
      beat_at_slave.wstrb = 0;
    }
  }
  
  sb_lock->unlock();
  
  wr_trans_generated++;
}; // End of Write generator

template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int MASTER_NUM, unsigned int SLAVE_NUM>
void ace_master<RD_M_LANES, RD_S_LANES, WR_M_LANES, WR_S_LANES, MASTER_NUM, SLAVE_NUM>::gen_new_cache_trans() {
  ace5_::AddrPayload cache_req;
  
  cache_req.id    = MASTER_ID;// (rand() % AXI_TID_NUM); // (rand()% 2)+2;
  cache_req.size  = nvhls::log2_ceil<RD_M_LANES>::val;
  cache_req.burst = enc_::AXBURST::INCR;
  cache_req.len   = 0;
  cache_req.addr  = ((rand()%ACE_CACHE_LINES)+1) * (ace5_::C_CACHE_WIDTH>>3);//0x8;
  
  cache_line & this_line = cache[cache_req.addr];
  
  cache_req.domain = enc_::AxDOMAIN::OUTER_SHARE;
  // RD_ONCE, RD_SHARED, RD_CLEAN, RD_NOT_SHARED_DIRTY, RD_UNIQUE, CLEAN_UNIQUE, MAKE_UNIQUE, CLEAN_SHARED, CLEAN_INVALID, MAKE_INVALID
  // WR_UNIQUE, WR_LINE_UNIQUE
  bool is_read = false;
  if (this_line.is_inv()) {
    unsigned sel_req = rand() % 11;
    if      (sel_req == 0) cache_req.snoop = enc_::ARSNOOP::RD_ONCE;
    else if (sel_req == 1) cache_req.snoop = enc_::ARSNOOP::RD_CLEAN;
    else if (sel_req == 2) cache_req.snoop = enc_::ARSNOOP::RD_NOT_SHARED_DIRTY;
    else if (sel_req == 3) cache_req.snoop = enc_::ARSNOOP::RD_SHARED;
    else if (sel_req == 4) cache_req.snoop = enc_::ARSNOOP::RD_UNIQUE;
    else if (sel_req == 5) cache_req.snoop = enc_::ARSNOOP::CLEAN_SHARED;
    else if (sel_req == 6) cache_req.snoop = enc_::ARSNOOP::CLEAN_INVALID;
    else if (sel_req == 7) cache_req.snoop = enc_::ARSNOOP::MAKE_UNIQUE;
    else if (sel_req == 8) {
      this_line.state = cache_line::State::INV;
      cache_req.snoop = enc_::ARSNOOP::MAKE_INVALID;
    }

    else if (sel_req == 9)  cache_req.snoop = enc_::AWSNOOP::WR_UNIQUE;
    else if (sel_req == 10) cache_req.snoop = enc_::AWSNOOP::WR_LINE_UNIQUE;
    is_read = (sel_req < 9);
  } else if (this_line.is_uc()) {
    unsigned sel_req = rand() % 3;
    if      (sel_req == 0) cache_req.snoop = enc_::ARSNOOP::CLEAN_SHARED;

    else if (sel_req == 1)cache_req.snoop = enc_::AWSNOOP::WR_UNIQUE;
    else if (sel_req == 2)cache_req.snoop = enc_::AWSNOOP::WR_LINE_UNIQUE;
    is_read = (sel_req < 1);
  } else if (this_line.is_ud()) {
    this_line.state = cache_line::State::INV;
    return;
  } else if (this_line.is_sc()) {
    unsigned sel_req = rand() % 5;
    // RD_ONCE, RD_SHARED, RD_CLEAN, RD_NOT_SHARED_DIRTY, RD_UNIQUE, CLEAN_UNIQUE, MAKE_UNIQUE, CLEAN_SHARED, CLEAN_INVALID, MAKE_INVALID
    if      (sel_req == 0) cache_req.snoop = enc_::ARSNOOP::CLEAN_UNIQUE;
    else if (sel_req == 1) cache_req.snoop = enc_::ARSNOOP::CLEAN_SHARED;
    else if (sel_req == 2) cache_req.snoop = enc_::ARSNOOP::MAKE_UNIQUE;

    else if (sel_req == 3) cache_req.snoop = enc_::AWSNOOP::WR_UNIQUE;
    else if (sel_req == 4) cache_req.snoop = enc_::AWSNOOP::WR_LINE_UNIQUE;
    is_read = (sel_req<3);
  } else if (this_line.is_sd()) {
    this_line.state = cache_line::State::INV;
    return;
    
    unsigned sel_req = rand() % 2;
    // RD_ONCE, RD_SHARED, RD_CLEAN, RD_NOT_SHARED_DIRTY, RD_UNIQUE, CLEAN_UNIQUE, MAKE_UNIQUE, CLEAN_SHARED, CLEAN_INVALID, MAKE_INVALID
    if      (sel_req == 0) cache_req.snoop = enc_::ARSNOOP::CLEAN_UNIQUE;
    else if (sel_req == 1) cache_req.snoop = enc_::ARSNOOP::MAKE_UNIQUE;
    is_read = false;
  } else {
    std::cout << "You should not reach this...\n";
    NVHLS_ASSERT(0);
  }
  
  if (true /*total_cycles > 14 && total_cycles <16*/) {
    if (is_read) {
      // Push it to injection queue
      stored_rd_trans.push(cache_req);
      
      // Pushing to ACE Coherency checker happens during injection
    
      // Push into order queue - Reorder check extension
      sb_rd_order_q.push_back(cache_req);
    
      rd_trans_generated++;
    } else {  // It's a WR request
      ace5_::WritePayload data_beat;
      if (ace5_::C_CACHE_WIDTH<64) data_beat.data = (((ace5_::Data) MASTER_ID) << (ace5_::C_CACHE_WIDTH - 8)) | ((ace5_::Data) 0x000000000000BEEF);
      else                         data_beat.data = (((ace5_::Data) MASTER_ID) << (ace5_::C_CACHE_WIDTH - 8)) | ((ace5_::Data) 0x0000BEEFDEADBEEF);
      data_beat.wstrb = -1;
      data_beat.last = 1;
    
      // Push it to injection queue
      stored_wr_trans.push(cache_req);
      stored_wr_data.push(data_beat);
    
      // Push it to Scoreboard
      sb_lock->lock();
      unsigned target_mem = mem_map_resolve(cache_req.addr);
      msg_tb_wrap<ace5_::AddrPayload> temp_wr_coherent_req_tb;
      temp_wr_coherent_req_tb.is_read         = false;
      temp_wr_coherent_req_tb.dut_msg         = cache_req;
      temp_wr_coherent_req_tb.dut_msg.snoop   = 0;
      temp_wr_coherent_req_tb.dut_msg.domain  = 0;
      temp_wr_coherent_req_tb.dut_msg.barrier = 0;
      temp_wr_coherent_req_tb.dut_msg.unique  = 0;
      temp_wr_coherent_req_tb.time_gen        = sc_time_stamp();
      (*sb_wr_req_q)[target_mem].push_back(temp_wr_coherent_req_tb);
    
      msg_tb_wrap<ace5_::WritePayload> temp_wr_data_tb;
      temp_wr_data_tb.dut_msg = data_beat;
      temp_wr_data_tb.is_read = false;
      temp_wr_data_tb.time_gen = sc_time_stamp();
    
      (*sb_wr_data_q)[target_mem].push_back(temp_wr_data_tb);
      sb_lock->unlock();
    
      // Push into order queue - Reorder check extension
      sb_wr_order_q.push_back(cache_req);
      wr_trans_generated++;
    }
    cache_trans_generated++;
  }
}; // End of Cache generator


template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int MASTER_NUM, unsigned int SLAVE_NUM>
void ace_master<RD_M_LANES, RD_S_LANES, WR_M_LANES, WR_S_LANES, MASTER_NUM, SLAVE_NUM>::gen_snoop_resp(ace5_::AC &rcv_snoop_req) {
  typename std::map<ace5_::Addr, cache_line>::iterator cur_line_iter;
  
  //  CRRESP[0] : DataTransfer
  //  CRRESP[1] : Error
  //  CRRESP[2] : PassDirty
  //  CRRESP[3] : IsShared
  //  CRRESP[4] : WasUnique
  cur_line_iter = cache.find(rcv_snoop_req.addr);
  ace5_::CR cur_resp;
  ace5_::CD cur_data;
  bool has_data = false;
  if (cur_line_iter == cache.end() || cur_line_iter->second.is_inv()) {
    std::cout << "[Master " << MASTER_ID << "] SNOOP Miss " << rcv_snoop_req << "@" << sc_time_stamp() << "\n";
    cur_resp.resp = 0;
    has_data = false;
  } else {
    std::cout << "[Master " << MASTER_ID << "] SNOOP Hit @" << sc_time_stamp() << " " << rcv_snoop_req;
    std::cout << " --- Addr:"<< std::hex << cur_line_iter->first << std::dec << " : " << cur_line_iter->second << "\n";
  
    cur_data.data = cur_line_iter->second.data;
    cur_data.last = 1;
    has_data = true;
  
    //  CRRESP[0] : DataTransfer
    //  CRRESP[1] : Error
    //  CRRESP[2] : PassDirty
    //  CRRESP[3] : IsShared
    //  CRRESP[4] : WasUnique
    if (rcv_snoop_req.snoop == enc_::ACSNOOP::RD_ONCE) {
      if (cur_line_iter->second.is_uc()) {
        cur_resp.resp = 0x19; // WasUnique, IsShared, DataTranfer
      } else if (cur_line_iter->second.is_ud()) {
        cur_resp.resp = 0x19; // WasUnique, IsShared, DataTranfer
      } else if (cur_line_iter->second.is_sc()) {
        cur_resp.resp = 0x9; // IsShared, DataTranfer
      } else if (cur_line_iter->second.is_sd()) {
        cur_resp.resp = 0x9; // IsShared, DataTranfer
      } else {
        std::cout << "You should not reach this...\n";
        NVHLS_ASSERT(0);
      }
    } else if ( (rcv_snoop_req.snoop == enc_::ACSNOOP::RD_CLEAN)  ||
                (rcv_snoop_req.snoop == enc_::ACSNOOP::RD_SHARED) ||
                (rcv_snoop_req.snoop == enc_::ACSNOOP::RD_NOT_SHARED_DIRTY) )
    {
      if (cur_line_iter->second.is_uc()) {
        cur_resp.resp = 0x19; // WasUnique, IsShared
        cur_line_iter->second.state = cache_line::State::SC;
      } else if (cur_line_iter->second.is_ud()) {
        cur_resp.resp = 0x19; // WasUnique, IsShared
        cur_line_iter->second.state = cache_line::State::SD;
      } else if (cur_line_iter->second.is_sc()) {
        cur_resp.resp = 0x9; // IsShared
      } else if (cur_line_iter->second.is_sd()) {
        cur_resp.resp = 0x9; // IsShared
        //cur_resp.resp = 0x5; // PassDirty                      // <---- THIS IS CHANGED
        //cur_line_iter->second.state = cache_line::State::INV;
      } else {
        std::cout << "You should not reach this...\n";
        NVHLS_ASSERT(0);
      }
    } else if ((rcv_snoop_req.snoop == enc_::ACSNOOP::RD_UNIQUE)     ||
               (rcv_snoop_req.snoop == enc_::ACSNOOP::CLEAN_INVALID) ||
               (rcv_snoop_req.snoop == enc_::ACSNOOP::MAKE_INVALID)    )
    {
      if      (cur_line_iter->second.is_uc()) cur_resp.resp = 0x10; // WasUnique
      else if (cur_line_iter->second.is_ud()) cur_resp.resp = 0x15; // WasUnique, PassDirty
      else if (cur_line_iter->second.is_sc()) cur_resp.resp = 0x00; //
      else if (cur_line_iter->second.is_sd()) cur_resp.resp = 0x05; // PassDirty
      else {
        std::cout << "You should not reach this...\n";
        NVHLS_ASSERT(0);
      }
      cur_line_iter->second.state = cache_line::State::INV;
    } else if (rcv_snoop_req.snoop == enc_::ACSNOOP::CLEAN_SHARED) {
      if (cur_line_iter->second.is_uc()) {
        cur_resp.resp = 0x19; // WasUnique, IsShared
        cur_line_iter->second.state = cache_line::State::UC;
      } else if (cur_line_iter->second.is_ud()) {
        cur_resp.resp = 0x1D; // WasUnique, IsShared
        cur_line_iter->second.state = cache_line::State::SC;
      } else if (cur_line_iter->second.is_sc()) {
        cur_resp.resp = 0x9; // IsShared
      } else if (cur_line_iter->second.is_sd()) {
        cur_resp.resp = 0xD; // IsShared
        cur_line_iter->second.state = cache_line::State::SC;
      } else {
        std::cout << "You should not reach this...\n";
        NVHLS_ASSERT(0);
      }
    }
  }
  
  has_data = cur_resp.resp & 1;
  stored_cache_resp.push(cur_resp);
  if(has_data) stored_cache_data.push(cur_data);
  
  // PUSH TO SCOREBOARD / COHERENCY CHECKER
  msg_tb_wrap<ace5_::AC> temp_snoop_req_tb;
  msg_tb_wrap<ace5_::CR> temp_snoop_resp_tb;
  msg_tb_wrap<ace5_::CD> temp_snoop_data_tb;
  
  sc_time now_time = sc_time_stamp();
  
  temp_snoop_req_tb.time_gen  = now_time;
  temp_snoop_req_tb.dut_msg   = rcv_snoop_req;
  
  temp_snoop_resp_tb.time_gen = now_time;
  temp_snoop_resp_tb.dut_msg  = cur_resp;
  
  temp_snoop_data_tb.time_gen = now_time;
  temp_snoop_data_tb.dut_msg  = cur_data;
  
  sb_lock->lock();
  (*sb_snoop_req_q)[MASTER_ID-SLAVE_NUM].push_back(temp_snoop_req_tb);
  if (has_data)  (*sb_snoop_data_resp_q)[MASTER_ID-SLAVE_NUM].push_back(temp_snoop_data_tb);
  (*sb_snoop_resp_q)[MASTER_ID-SLAVE_NUM].push_back(temp_snoop_resp_tb); // Keep pushing to resp last, just to be ULTRA sure for read access sequence. (coherency checker checks resp to read the req-resp-data set)
  sb_lock->unlock();
}

template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int MASTER_NUM, unsigned int SLAVE_NUM>
void ace_master<RD_M_LANES, RD_S_LANES, WR_M_LANES, WR_S_LANES, MASTER_NUM, SLAVE_NUM>::upd_cache_read(ace5_::AddrPayload req, ace5_::ReadPayload resp) {
  cache_line & this_line = cache[req.addr];
   unsigned shared_dirty = (resp.resp) >> 2;
  
  // --- READ Operations --- //
  if (req.snoop == enc_::ARSNOOP::RD_ONCE && req.domain.xor_reduce()) {
    if (this_line.is_inv()) { // Expected start state
      if ( (shared_dirty != 0) &&
           (shared_dirty != 2) )
      { NVHLS_ASSERT_MSG(0, "Invalid response.");}
      this_line.state = cache_line::State::INV;
      this_line.data  = resp.data;
    } else if (this_line.is_uc()) {
      std::cout << "WARN : Cache in UC not expected (though permitted) for Req: " << req << "\n";
      if ( (shared_dirty != 0) &&
           (shared_dirty != 2) )
      { NVHLS_ASSERT_MSG(0, "Invalid response.");}
      this_line.state = cache_line::State::UC;
      this_line.data  = resp.data;
    } else if (this_line.is_ud()) {
      std::cout << "WARN : Cache in UD not expected (though permitted) for Req: " << req << "\n";
      if ( (shared_dirty != 0) &&
           (shared_dirty != 2) )
      { NVHLS_ASSERT_MSG(0, "Invalid response.");}
      this_line.state = cache_line::State::UD;
      this_line.data  = resp.data;
    } else if (this_line.is_sc()) {
      std::cout << "WARN : Cache in SC not expected (though permitted) for Req: " << req << "\n";
      if ( (shared_dirty != 0) &&
           (shared_dirty != 2) )
      { NVHLS_ASSERT_MSG(0, "Invalid response.");}
      this_line.state = (shared_dirty==0) ? cache_line::State::UC : cache_line::State::SC;
      this_line.data  = resp.data;
    } else if (this_line.is_sd()) {
      std::cout << "WARN : Cache in SD not expected (though permitted) for Req: " << req << "\n";
      if ( (shared_dirty != 0) &&
           (shared_dirty != 2) )
      { NVHLS_ASSERT_MSG(0, "Invalid response.");}
      this_line.state = (shared_dirty==0) ? cache_line::State::UD : cache_line::State::SD;
      this_line.data  = resp.data;
    } else {
      std::cout << "Impossible to reach this. \n";
      NVHLS_ASSERT(0);
    }
  } else if (req.snoop == enc_::ARSNOOP::RD_CLEAN) {
    if (this_line.is_inv()) { // Expected start state
      if ( (shared_dirty != 0) &&
           (shared_dirty != 2) )
      { NVHLS_ASSERT_MSG(0, "Invalid response.");}
      this_line.state = (shared_dirty==0) ? cache_line::State::UC : cache_line::State::SC;
      this_line.data  = resp.data;
    } else if (this_line.is_uc()) {
      std::cout << "WARN : Cache in UC not expected (though permitted) for Req: " << req << "\n";
      if ( (shared_dirty != 0) )
      { NVHLS_ASSERT_MSG(0, "Invalid response.");}
      this_line.state = cache_line::State::UC;
      this_line.data  = resp.data;
    } else if (this_line.is_ud()) {
      std::cout << "WARN : Cache in UD not expected (though permitted) for Req: " << req << "\n";
      if ( (shared_dirty != 0) )
      { NVHLS_ASSERT_MSG(0, "Invalid response.");}
      this_line.state = cache_line::State::UD;
      this_line.data  = resp.data;
    } else if (this_line.is_sc()) {
      std::cout << "WARN : Cache in SC not expected (though permitted) for Req: " << req << "\n";
      if ( (shared_dirty != 0) &&
         (shared_dirty != 2) )
      { NVHLS_ASSERT_MSG(0, "Invalid response.");}
      this_line.state = (shared_dirty==0) ? cache_line::State::UC : cache_line::State::SC;
      this_line.data  = resp.data;
    } else if (this_line.is_sd()) {
      std::cout << "WARN : Cache in SD not expected (though permitted) for Req: " << req << "\n";
      if ( (shared_dirty != 0) &&
           (shared_dirty != 2) )
      { NVHLS_ASSERT_MSG(0, "Invalid response.");}
      this_line.state = (shared_dirty==0) ? cache_line::State::UD : cache_line::State::SD;
      this_line.data  = resp.data;
    } else {
      std::cout << "Impossible to reach this. \n";
      NVHLS_ASSERT(0);
    }
  } else if (req.snoop == enc_::ARSNOOP::RD_NOT_SHARED_DIRTY) {
    if (this_line.is_inv()) { // Expected start state
      if ( (shared_dirty != 0) &&
           (shared_dirty != 1) &&
           (shared_dirty != 2) )
      { NVHLS_ASSERT_MSG(0, "Invalid response.");}
      this_line.state = (shared_dirty==0) ? cache_line::State::UC :
                        (shared_dirty==1) ? cache_line::State::UD
                                          : cache_line::State::SC;
      this_line.data  = resp.data;
    } else if (this_line.is_uc()) {
      std::cout << "WARN : Cache in UC not expected (though permitted) for Req: " << req << "\n";
      if ( (shared_dirty != 0) )
      { NVHLS_ASSERT_MSG(0, "Invalid response.");}
      this_line.state = cache_line::State::UC;
      this_line.data  = resp.data;
    } else if (this_line.is_ud()) {
      std::cout << "WARN : Cache in UD not expected (though permitted) for Req: " << req << "\n";
      if ( (shared_dirty != 0) )
      { NVHLS_ASSERT_MSG(0, "Invalid response.");}
      this_line.state = cache_line::State::UD;
      this_line.data  = resp.data;
    } else if (this_line.is_sc()) {
      std::cout << "WARN : Cache in SC not expected (though permitted) for Req: " << req << "\n";
      if ( (shared_dirty != 0) &&
           (shared_dirty != 1) &&
           (shared_dirty != 2) )
      { NVHLS_ASSERT_MSG(0, "Invalid response.");}
      this_line.state = (shared_dirty==0) ? cache_line::State::UC :
                        (shared_dirty==1) ? cache_line::State::UD
                                          : cache_line::State::SC;
      this_line.data  = resp.data;
    } else if (this_line.is_sd()) {
      std::cout << "WARN : Cache in SD not expected (though permitted) for Req: " << req << "\n";
      if ( (shared_dirty != 0) &&
           (shared_dirty != 2) )
      { NVHLS_ASSERT_MSG(0, "Invalid response.");}
      this_line.state = (shared_dirty==0) ? cache_line::State::UD : cache_line::State::SD;
      this_line.data  = resp.data;
    } else {
      std::cout << "Impossible to reach this. \n";
      NVHLS_ASSERT(0);
    }
  } else if (req.snoop == enc_::ARSNOOP::RD_SHARED) {
    if (this_line.is_inv()) { // Expected start state
      if ( (shared_dirty != 0) &&
           (shared_dirty != 1) &&
           (shared_dirty != 2) &&
           (shared_dirty != 3) )
      { NVHLS_ASSERT_MSG(0, "Invalid response.");}
      this_line.state = (shared_dirty==0) ? cache_line::State::UC :
                        (shared_dirty==1) ? cache_line::State::UD :
                        (shared_dirty==2) ? cache_line::State::SC
                                          : cache_line::State::SD;
      this_line.data  = resp.data;
    } else if (this_line.is_uc()) {
      std::cout << "WARN : Cache in UC not expected (though permitted) for Req: " << req << "\n";
      if ( (shared_dirty != 0) )
      { NVHLS_ASSERT_MSG(0, "Invalid response.");}
      this_line.state = cache_line::State::UC;
      this_line.data  = resp.data;
    } else if (this_line.is_ud()) {
      std::cout << "WARN : Cache in UD not expected (though permitted) for Req: " << req << "\n";
      if ( (shared_dirty != 0) )
      { NVHLS_ASSERT_MSG(0, "Invalid response.");}
      this_line.state = cache_line::State::UD;
      this_line.data  = resp.data;
    } else if (this_line.is_sc()) {
      std::cout << "WARN : Cache in SC not expected (though permitted) for Req: " << req << "\n";
      if ( (shared_dirty != 0) &&
           (shared_dirty != 1) &&
           (shared_dirty != 2) &&
           (shared_dirty != 3) )
      { NVHLS_ASSERT_MSG(0, "Invalid response.");}
      this_line.state = (shared_dirty==0) ? cache_line::State::UC :
                        (shared_dirty==1) ? cache_line::State::UD :
                        (shared_dirty==2) ? cache_line::State::SC
                                          : cache_line::State::SD;
      this_line.data  = resp.data;
    } else if (this_line.is_sd()) {
      std::cout << "WARN : Cache in SD not expected (though permitted) for Req: " << req << "\n";
      if ( (shared_dirty != 0) &&
           (shared_dirty != 2) )
      { NVHLS_ASSERT_MSG(0, "Invalid response.");}
      this_line.state = (shared_dirty==0) ? cache_line::State::UD : cache_line::State::SD;
      this_line.data  = resp.data;
    } else {
      std::cout << "Impossible to reach this. \n";
      NVHLS_ASSERT(0);
    }
  } else if (req.snoop == enc_::ARSNOOP::RD_UNIQUE) {
    if (this_line.is_inv()) { // Expected start state
      if ( (shared_dirty != 0) &&
           (shared_dirty != 1) )
      { NVHLS_ASSERT_MSG(0, "Invalid response.");}
      this_line.state = (shared_dirty==0) ? cache_line::State::UC : cache_line::State::UD;
      this_line.data  = resp.data;
    } else if (this_line.is_uc()) {
      std::cout << "WARN : Cache in UC not expected (though permitted) for Req: " << req << "\n";
      if ( (shared_dirty != 0) )
      { NVHLS_ASSERT_MSG(0, "Invalid response.");}
      this_line.state = cache_line::State::UC;
      this_line.data  = resp.data;
    } else if (this_line.is_ud()) {
      std::cout << "WARN : Cache in UD not expected (though permitted) for Req: " << req << "\n";
      if ( (shared_dirty != 0) )
      { NVHLS_ASSERT_MSG(0, "Invalid response.");}
      this_line.state = cache_line::State::UD;
      this_line.data  = resp.data;
    } else if (this_line.is_sc()) {
      std::cout << "WARN : Cache in SC not expected (though permitted) for Req: " << req << "\n";
      if ( (shared_dirty != 0) &&
           (shared_dirty != 1) )
      { NVHLS_ASSERT_MSG(0, "Invalid response.");}
      this_line.state = (shared_dirty==0) ? cache_line::State::UC : cache_line::State::UD;
      this_line.data  = resp.data;
    } else if (this_line.is_sd()) {
      std::cout << "WARN : Cache in SD not expected (though permitted) for Req: " << req << "\n";
      if ( (shared_dirty != 0) )
      { NVHLS_ASSERT_MSG(0, "Invalid response.");}
      this_line.state = cache_line::State::UD;
      this_line.data  = resp.data;
    } else {
      std::cout << "Impossible to reach this. \n";
      NVHLS_ASSERT(0);
    }
  // --- CLEAN Operations --- //
  } else if (req.snoop == enc_::ARSNOOP::CLEAN_UNIQUE) {
    if (this_line.is_inv()) {
      std::cout << "WARN : Cache in INV not expected (though permitted) for Req: " << req << "\n";
      if ( (shared_dirty != 0) )
      { NVHLS_ASSERT_MSG(0, "Invalid response.");}
      this_line.state = cache_line::State::INV;
    } else if (this_line.is_uc()) {
      std::cout << "WARN : Cache in UC not expected (though permitted) for Req: " << req << "\n";
      if ( (shared_dirty != 0) )
      { NVHLS_ASSERT_MSG(0, "Invalid response.");}
      this_line.state = cache_line::State::UC;
    } else if (this_line.is_ud()) {
      std::cout << "WARN : Cache in UD not expected (though permitted) for Req: " << req << "\n";
      if ( (shared_dirty != 0) )
      { NVHLS_ASSERT_MSG(0, "Invalid response.");}
      this_line.state = cache_line::State::UD;
    } else if (this_line.is_sc()) { // Expected start state
      if ( (shared_dirty != 0) )
      { NVHLS_ASSERT_MSG(0, "Invalid response.");}
      this_line.state = cache_line::State::UC;
    } else if (this_line.is_sd()) { // Expected start state
      if ( (shared_dirty != 0) )
      { NVHLS_ASSERT_MSG(0, "Invalid response.");}
      this_line.state = cache_line::State::UD;
    } else {
      std::cout << "Impossible to reach this. \n";
      NVHLS_ASSERT(0);
    }
  } else if (req.snoop == enc_::ARSNOOP::CLEAN_SHARED) {
    if (this_line.is_inv()) { // Expected start state
      if ( (shared_dirty != 0) &&
           (shared_dirty != 2) )
      { NVHLS_ASSERT_MSG(0, "Invalid response.");}
      this_line.state = cache_line::State::INV;
    } else if (this_line.is_uc()) { // Expected start state
      if ( (shared_dirty != 0) )
      { NVHLS_ASSERT_MSG(0, "Invalid response.");}
      this_line.state = cache_line::State::UC;
    } else if (this_line.is_ud()) {
      std::cout << "WARN : Cache is not in expected state for this Request.\n";
      //NVHLS_ASSERT(0);
    } else if (this_line.is_sc()) {
      if ( (shared_dirty != 0) &&
           (shared_dirty != 2) )
      { NVHLS_ASSERT_MSG(0, "Invalid response.");}
      this_line.state = (shared_dirty==0) ? cache_line::State::UC : cache_line::State::SC;
    } else if (this_line.is_sd()) {
      std::cout << "CHACHE MODEL ERR : Cache is not in expected state for this Request.\n";
      //NVHLS_ASSERT(0);
    } else {
      std::cout << "Impossible to reach this. \n";
      NVHLS_ASSERT(0);
    }
  } else if (req.snoop == enc_::ARSNOOP::CLEAN_INVALID) {
    // ToDo this is a phony state INVALIDATION. Because It's cache's responsibility to INV line before a CLEAN_INVALID.
    // Workaround until a valid cache controller model is available
    this_line.state = cache_line::State::INV;
    if (this_line.is_inv()) { // Expected start state
      if ( (shared_dirty != 0) )
      { NVHLS_ASSERT_MSG(0, "Invalid response.");}
      this_line.state = cache_line::State::INV;
    } else if (this_line.is_uc()) {
      std::cout << "ERR : Cache is not in expected state for this Request.\n";
      NVHLS_ASSERT(0);
    } else if (this_line.is_ud()) {
      std::cout << "ERR : Cache is not in expected state for this Request.\n";
      NVHLS_ASSERT(0);
    } else if (this_line.is_sc()) {
      std::cout << "ERR : Cache is not in expected state for this Request.\n";
      NVHLS_ASSERT(0);
    } else if (this_line.is_sd()) {
      std::cout << "ERR : Cache is not in expected state for this Request.\n";
      NVHLS_ASSERT(0);
    } else {
      std::cout << "Impossible to reach this. \n";
      NVHLS_ASSERT(0);
    }
  } else if (req.snoop == enc_::ARSNOOP::MAKE_UNIQUE) {
    if (this_line.is_inv()) { // Expected start state
      if ( (shared_dirty != 0) )
      { NVHLS_ASSERT_MSG(0, "Invalid response.");}
      this_line.state = cache_line::State::UD;
    } else if (this_line.is_uc()) {
      std::cout << "WARN : Cache in UC not expected (though permitted) for Req: " << req << "\n";
      if ( (shared_dirty != 0) )
      { NVHLS_ASSERT_MSG(0, "Invalid response.");}
      this_line.state = cache_line::State::UD;
    } else if (this_line.is_ud()) {
      std::cout << "WARN : Cache in UD not expected (though permitted) for Req: " << req << "\n";
      if ( (shared_dirty != 0) )
      { NVHLS_ASSERT_MSG(0, "Invalid response.");}
      this_line.state = cache_line::State::UD;
    } else if (this_line.is_sc()) { // Expected start state
      if ( (shared_dirty != 0) )
      { NVHLS_ASSERT_MSG(0, "Invalid response.");}
      this_line.state = cache_line::State::UD;
    } else if (this_line.is_sd()) { // Expected start state
      if ( (shared_dirty != 0) )
      { NVHLS_ASSERT_MSG(0, "Invalid response.");}
      this_line.state = cache_line::State::UD;
    } else {
      std::cout << "Impossible to reach this. \n";
      NVHLS_ASSERT(0);
    }
  } else if (req.snoop == enc_::ARSNOOP::MAKE_INVALID) {
    // ToDo this is a phony state INVALIDATION. Because It's cache's responsibility to INV line before a MAKE_INVALID.
    // Workaround until a valid cache controller model is available
    this_line.state = cache_line::State::INV;
    if (this_line.is_inv()) { // Expected start state
      if ( (shared_dirty != 0) )
      { NVHLS_ASSERT_MSG(0, "Invalid response.");}
      this_line.state = cache_line::State::INV;
    } else if (this_line.is_uc()) {
      std::cout << "ERR : Cache is not in expected state for this Request.\n";
      NVHLS_ASSERT(0);
    } else if (this_line.is_ud()) {
      std::cout << "ERR : Cache is not in expected state for this Request.\n";
      NVHLS_ASSERT(0);
    } else if (this_line.is_sc()) {
      std::cout << "ERR : Cache is not in expected state for this Request.\n";
      NVHLS_ASSERT(0);
    } else if (this_line.is_sd()) {
      std::cout << "ERR : Cache is not in expected state for this Request.\n";
      NVHLS_ASSERT(0);
    } else {
      std::cout << "Impossible to reach this. \n";
      NVHLS_ASSERT(0);
    }
  } else {
    std::cout << "  Req : " << req <<". \n";
    std::cout << "  Resp: " << resp <<". \n";
    std::cout << "  Did not handled correctly. \n";
    NVHLS_ASSERT(0);
  }
  
  cache_outstanding[req.addr]--;
  
  // when end-up in a Dirty state write The master ID
  if (this_line.is_ud() || this_line.is_sd()) {
    if (ace5_::C_CACHE_WIDTH<64) this_line.data = (((ace5_::Data) 0xFF) << (ace5_::C_CACHE_WIDTH - 8)) | ((ace5_::Data) 0x000000000000BEEF);
    else                         this_line.data = (((ace5_::Data) 0xFF) << (ace5_::C_CACHE_WIDTH - 8)) | ((ace5_::Data) 0x0000BEEFDEADBEEF);
    //this_line.data  = ( ((ace5_::Data) MASTER_ID) << (ace5_::C_CACHE_WIDTH-8) ) | 0x0000BEEFDEADBEEF;
    //this_line.data = (this_line.data & ( (~((ace5_::Data) 0xFF) << (ace5_::C_CACHE_WIDTH-8)) ) ) | ( ((ace5_::Data) MASTER_ID) << (ace5_::C_CACHE_WIDTH-8) );
  }
  
}

template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int MASTER_NUM, unsigned int SLAVE_NUM>
void ace_master<RD_M_LANES, RD_S_LANES, WR_M_LANES, WR_S_LANES, MASTER_NUM, SLAVE_NUM>::upd_cache_write(ace5_::AddrPayload req, ace5_::WRespPayload resp) {
  cache_line &this_line = cache[req.addr];
  
  if ((req.snoop == enc_::AWSNOOP::WR_UNIQUE && req.domain.xor_reduce()) ||
       req.snoop == enc_::AWSNOOP::WR_LINE_UNIQUE)
  {
    if (this_line.is_inv()) {
      this_line.state = cache_line::State::INV;
    } else if (this_line.is_uc()) {
      this_line.state = cache_line::State::SC;
      if (ace5_::C_CACHE_WIDTH<64) this_line.data = (((ace5_::Data) 0xFF) << (ace5_::C_CACHE_WIDTH - 8)) | ((ace5_::Data) 0x000000000000BEEF);
      else                         this_line.data = (((ace5_::Data) 0xFF) << (ace5_::C_CACHE_WIDTH - 8)) | ((ace5_::Data) 0x0000BEEFDEADBEEF);
    } else if (this_line.is_ud()) {
      std::cout << "WARN : Cache is not in expected state for this Request.\n";
      //NVHLS_ASSERT(0);
    } else if (this_line.is_sc()) {
      this_line.state = cache_line::State::SC;
      if (ace5_::C_CACHE_WIDTH<64) this_line.data = (((ace5_::Data) 0xFF) << (ace5_::C_CACHE_WIDTH - 8)) | ((ace5_::Data) 0x000000000000BEEF);
      else                         this_line.data = (((ace5_::Data) 0xFF) << (ace5_::C_CACHE_WIDTH - 8)) | ((ace5_::Data) 0x0000BEEFDEADBEEF);
    } else if (this_line.is_sd()) {
      std::cout << "WARN : Cache is not in expected state for this Request.\n";
      //NVHLS_ASSERT(0);
    } else {
      std::cout << "Impossible to reach this. \n";
      NVHLS_ASSERT(0);
    }
  } else if (req.snoop == enc_::AWSNOOP::WR_BACK) {
    if (this_line.is_inv()) {
      std::cout << "ERR : Cache is not in expected state for this Request.\n";
      NVHLS_ASSERT(0);
    } else if (this_line.is_uc()) {
      std::cout << "ERR : Cache is not in expected state for this Request.\n";
      NVHLS_ASSERT(0);
    } else if (this_line.is_ud()) {
      this_line.state = cache_line::State::INV;
    } else if (this_line.is_sc()) {
      std::cout << "ERR : Cache is not in expected state for this Request.\n";
      NVHLS_ASSERT(0);
    } else if (this_line.is_sd()) {
      this_line.state = cache_line::State::INV;
    } else {
      std::cout << "Impossible to reach this. \n";
      NVHLS_ASSERT(0);
    }
  } else if (req.snoop == enc_::AWSNOOP::WR_CLEAN) {
    if (this_line.is_inv()) {
      std::cout << "ERR : Cache is not in expected state for this Request.\n";
      NVHLS_ASSERT(0);
    } else if (this_line.is_uc()) {
      std::cout << "ERR : Cache is not in expected state for this Request.\n";
      NVHLS_ASSERT(0);
    } else if (this_line.is_ud()) {
      this_line.state = cache_line::State::INV;
    } else if (this_line.is_sc()) {
      std::cout << "ERR : Cache is not in expected state for this Request.\n";
      NVHLS_ASSERT(0);
    } else if (this_line.is_sd()) {
      this_line.state = cache_line::State::INV;
    } else {
      std::cout << "Impossible to reach this. \n";
      NVHLS_ASSERT(0);
    }
  } else if (req.snoop == enc_::AWSNOOP::WR_CLEAN) {
    if (this_line.is_inv()) {
      std::cout << "ERR : Cache is not in expected state for this Request.\n";
      NVHLS_ASSERT(0);
    } else if (this_line.is_uc()) {
      this_line.state = cache_line::State::INV;
    } else if (this_line.is_ud()) {
      std::cout << "ERR : Cache is not in expected state for this Request.\n";
      NVHLS_ASSERT(0);
    } else if (this_line.is_sc()) {
      std::cout << "ERR : Cache is not in expected state for this Request.\n";
      NVHLS_ASSERT(0);
    } else if (this_line.is_sd()) {
      std::cout << "ERR : Cache is not in expected state for this Request.\n";
      NVHLS_ASSERT(0);
    } else {
      std::cout << "Impossible to reach this. \n";
      NVHLS_ASSERT(0);
    }
  } else if (req.snoop == enc_::AWSNOOP::WR_EVICT) {
    if (this_line.is_inv()) {
      std::cout << "ERR : Cache is not in expected state for this Request.\n";
      NVHLS_ASSERT(0);
    } else if (this_line.is_uc()) {
      this_line.state = cache_line::State::INV;
    } else if (this_line.is_ud()) {
      std::cout << "ERR : Cache is not in expected state for this Request.\n";
      NVHLS_ASSERT(0);
    } else if (this_line.is_sc()) {
      std::cout << "ERR : Cache is not in expected state for this Request.\n";
      NVHLS_ASSERT(0);
    } else if (this_line.is_sd()) {
      std::cout << "ERR : Cache is not in expected state for this Request.\n";
      NVHLS_ASSERT(0);
    } else {
      std::cout << "Impossible to reach this. \n";
      NVHLS_ASSERT(0);
    }
  } else if (req.snoop == enc_::AWSNOOP::EVICT) {
    if (this_line.is_inv()) {
      std::cout << "ERR : Cache is not in expected state for this Request.\n";
      NVHLS_ASSERT(0);
    } else if (this_line.is_uc()) {
      this_line.state = cache_line::State::INV;
    } else if (this_line.is_ud()) {
      std::cout << "ERR : Cache is not in expected state for this Request.\n";
      NVHLS_ASSERT(0);
    } else if (this_line.is_sc()) {
      this_line.state = cache_line::State::INV;
    } else if (this_line.is_sd()) {
      std::cout << "ERR : Cache is not in expected state for this Request.\n";
      NVHLS_ASSERT(0);
    } else {
      std::cout << "Impossible to reach this. \n";
      NVHLS_ASSERT(0);
    }
  } else {
    std::cout << "  Req : " << req <<". \n";
    std::cout << "  Resp: " << resp <<". \n";
    std::cout << "  Did not handled correctly. \n";
    NVHLS_ASSERT(0);
  }
  cache_outstanding_writes[req.addr]--;
  cache_outstanding[req.addr]--;
}

// ------------------------ //
// --- VERIFY Functions --- //
// ------------------------ //
template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int MASTER_NUM, unsigned int SLAVE_NUM>
void ace_master<RD_M_LANES, RD_S_LANES, WR_M_LANES, WR_S_LANES, MASTER_NUM, SLAVE_NUM>::verify_rd_resp(ace5_::ReadPayload &rcv_rd_resp){
  // Verify Response
  sb_lock->lock();
  bool is_coherent = false;
  // --- Reorder Check --- //
  unsigned reorder=2; // 2 : Req not found, 1 : Request reordered, 0 : everything is fine
  unsigned j=0;
  ace5_::AddrPayload sb_ord_req;
  while (j<sb_rd_order_q.size()){
    sb_ord_req = sb_rd_order_q[j];
    
    if(sb_ord_req.id == rcv_rd_resp.id) {
      // Slave must sneak its ID to the resp field.
      unsigned dst = mem_map_resolve(sb_ord_req.addr);
      is_coherent = (sb_ord_req.snoop || sb_ord_req.domain.xor_reduce());
      reorder = (dst  == rcv_rd_resp.resp) || is_coherent ? 0 : 1;
      if(rcv_rd_resp.last) sb_rd_order_q.erase(sb_rd_order_q.begin()+j);
      break;
    }
    j++;
  }
  // --------------------- //
  bool found = false;
  j=0;
  //if (sb_ord_req.snoop == 0) {
    while (j<(*sb_rd_resp_q)[MASTER_ID-SLAVE_NUM].size()){
      msg_tb_wrap< ace5_::ReadPayload > sb_resp = (*sb_rd_resp_q)[MASTER_ID-SLAVE_NUM][j];
    
      if (eq_rd_data(rcv_rd_resp, sb_resp.dut_msg)){
        if (sb_resp.dut_msg.last) {
          rd_resp_delay += ((sc_time_stamp() - sb_resp.time_gen) / clk_period) - 1;
          rd_resp_count++;
        }
        rd_resp_data_count++;
        last_rd_sinked_cycle = (sc_time_stamp() / clk_period);
      
        (*sb_rd_resp_q)[MASTER_ID-SLAVE_NUM].erase((*sb_rd_resp_q)[MASTER_ID-SLAVE_NUM].begin()+j);
        found = true;
        break;
      }
      j++;
    }
  //} else {
  //  found = true; // Ignore the check if it's a Snoop access
  //}
  if (rcv_rd_resp.last) {
    ace5_::RACK tmp_rack;
    tmp_rack.rack = 1;
    stored_rd_ack.push(tmp_rack);
  }
  // --- DEPRECATED --- This checks absolute order among all TIDs
  //AXI4_R sb_val = (*sb_rd_resp_q)[MASTER_ID].front();
  //bool found = (rcv_rd_resp == sb_val);
  //if(found) (*sb_rd_resp_q)[MASTER_ID].erase((*sb_rd_resp_q)[MASTER_ID].begin());
  
  if(!found){
    std::cout<< "\n\n";
    std::cout<< "[Master " << MASTER_ID <<"] " << "RD-Resp  : "<< rcv_rd_resp << " . NOT FOUND! @" << sc_time_stamp() << "\n";
    std::cout<< "[Master " << MASTER_ID <<"] " << "-SB_front - "<< (*sb_rd_resp_q)[MASTER_ID-SLAVE_NUM].front() << "\n";
    error_sb_rd_resp_not_found++;
    sc_assert(0);
    // sc_stop();
  }else if(reorder==2) {
    std::cout<< "\n\n";
    std::cout<< "[Master " << MASTER_ID <<"] " << "RD-Resp  : "<< rcv_rd_resp << " . Respective Request wasn't found!!! @" << sc_time_stamp() << "\n";
    std::cout<< "[Master " << MASTER_ID <<"] " << "-REQ_front - "<< sb_rd_order_q.front() << "\n";
    sc_assert(0);
  }else if(reorder==1) {
    std::cout<< "\n\n";
    std::cout<< "[Master " << MASTER_ID <<"] " << "RD-Resp  : "<< rcv_rd_resp << " . Got Reordered !!! @" << sc_time_stamp() << "\n";
    std::cout<< "[Master " << MASTER_ID <<"] " << "REQ-Ordered - "<< sb_ord_req << "\n";
    sc_assert(0);
  }else{
    std::cout<< "[Master " << MASTER_ID <<"] " << "RD-Resp OK   : <<  " << rcv_rd_resp << " @" << sc_time_stamp() << "\n";
    if (is_coherent){
      upd_cache_read(sb_ord_req, rcv_rd_resp);
    } else {
      unsigned dbg_non_coh = 0;
    }
    rd_resp_ej++;
  }
  std::cout.flush();
  sb_lock->unlock();
}; // End of READ Response Verify

template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int MASTER_NUM, unsigned int SLAVE_NUM>
void ace_master<RD_M_LANES, RD_S_LANES, WR_M_LANES, WR_S_LANES, MASTER_NUM, SLAVE_NUM>::verify_wr_resp(ace5_::WRespPayload &rcv_wr_resp){
  sb_lock->lock();
  bool is_coherent = false;
  // --- Reorder Check --- //
  int reorder = 2; // 2 : Req not found, 1 : Request reordered, 0 : everything is fine
  unsigned int j = 0;
  ace5_::AddrPayload sb_ord_req;
  while (j<sb_wr_order_q.size()) {
    sb_ord_req = sb_wr_order_q[j];
    
    if(sb_ord_req.id == rcv_wr_resp.id) {
      // Slave must sneak its ID into the first data byte of every beat (aka data[0]).
      unsigned dst = mem_map_resolve(sb_ord_req.addr);
      is_coherent = (sb_ord_req.snoop || sb_ord_req.domain.xor_reduce());
      reorder = (dst  == rcv_wr_resp.resp) ? 0 : 1;
      sb_wr_order_q.erase(sb_wr_order_q.begin()+j);
      break;
    }
    
    j++;
  }
  // --------------------- //
  
  // Verify Responce
  bool found = false;
  j = 0;
  while (j<(*sb_wr_resp_q)[MASTER_ID-SLAVE_NUM].size()){
    msg_tb_wrap< ace5_::WRespPayload > sb_resp = (*sb_wr_resp_q)[MASTER_ID-SLAVE_NUM][j];
    
    if (eq_wr_resp(sb_resp.dut_msg, rcv_wr_resp)){
      
      wr_resp_delay += ((sc_time_stamp() - sb_resp.time_gen) / clk_period) - 1;
      wr_resp_count++;
      
      (*sb_wr_resp_q)[MASTER_ID-SLAVE_NUM].erase((*sb_wr_resp_q)[MASTER_ID-SLAVE_NUM].begin()+j);
      found = true;
      break;
    }
    j++;
  }
  
  //if (rcv_wr_resp.last) {
    ace5_::WACK tmp_wack;
    tmp_wack.wack = 1;
    stored_wr_ack.push(tmp_wack);
  //}
  
  if(!found){
    std::cout<< "\n\n";
    std::cout<< "[Master " << MASTER_ID <<"] " << "WR-Resp  : "<< rcv_wr_resp << " . NOT FOUND! @" << sc_time_stamp() << "\n";
    std::cout<< "[Master " << MASTER_ID <<"] " << "-SB_front - "<< (*sb_wr_resp_q)[MASTER_ID-SLAVE_NUM].front() << "\n";
    error_sb_wr_resp_not_found++;
    sc_assert(0);
    // sc_stop();
  }else if(reorder==2) {
    std::cout<< "\n\n";
    std::cout<< "[Master " << MASTER_ID <<"] " << "WR-Resp  : "<< rcv_wr_resp << " . Respective Request wasn't found!!! @" << sc_time_stamp() << "\n";
    std::cout<< "[Master " << MASTER_ID <<"] " << "-REQ_front - "<< sb_wr_order_q.front() << "\n";
    sc_assert(0);
  }else if(reorder==1) {
    std::cout<< "\n\n";
    std::cout<< "[Master " << MASTER_ID <<"] " << "WR-Resp  : "<< rcv_wr_resp << " . Got Reordered !!! @" << sc_time_stamp() << "\n";
    std::cout<< "[Master " << MASTER_ID <<"] " << "REQ-Ordered - "<< sb_ord_req << "\n";
    sc_assert(0);
  }else{
    std::cout<< "[Master " << MASTER_ID <<"] " << "WR-Resp OK   : <<  " << rcv_wr_resp << "\n";
    if (is_coherent) {
      upd_cache_write(sb_ord_req, rcv_wr_resp);
    } else {
      unsigned dbg_non_coh = 0;
    }
    wr_resp_ej++;
  }
  std::cout.flush();
  sb_lock->unlock();
}; // End of WRITE Response Verify


template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int MASTER_NUM, unsigned int SLAVE_NUM>
bool ace_master<RD_M_LANES, RD_S_LANES, WR_M_LANES, WR_S_LANES, MASTER_NUM, SLAVE_NUM>::eq_rd_data (ace5_::ReadPayload &rcv_rd_data, ace5_::ReadPayload &sb_rd_data) {
  unsigned tid_mask = (1<<dnp::ace::ID_W)-1;
  return ((rcv_rd_data.id & tid_mask) == (sb_rd_data.id & tid_mask) &&
           rcv_rd_data.data  == sb_rd_data.data  &&
           rcv_rd_data.resp  == sb_rd_data.resp  &&
           rcv_rd_data.last  == sb_rd_data.last  //&&
           //rcv_rd_data.ruser == sb_rd_data.ruser
  );
};

template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int MASTER_NUM, unsigned int SLAVE_NUM>
bool ace_master<RD_M_LANES, RD_S_LANES, WR_M_LANES, WR_S_LANES, MASTER_NUM, SLAVE_NUM>::eq_wr_resp (ace5_::WRespPayload &rcv_wr_resp, ace5_::WRespPayload &sb_wr_resp) {
  unsigned tid_mask = (1<<dnp::ace::ID_W)-1;
  return ((rcv_wr_resp.id & tid_mask) == (sb_wr_resp.id & tid_mask) &&
           rcv_wr_resp.resp  == sb_wr_resp.resp  //&&
           //rcv_wr_resp.buser == sb_wr_resp.buser
  );
};

template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int MASTER_NUM, unsigned int SLAVE_NUM>
unsigned ace_master<RD_M_LANES, RD_S_LANES, WR_M_LANES, WR_S_LANES, MASTER_NUM, SLAVE_NUM>::mem_map_resolve(ace5_::Addr &addr){
  for (int i=0; i<SLAVE_NUM; ++i) {
    if (addr>=addr_map[i][0].read() && addr <= addr_map[i][1].read()) return i;
  }
  NVHLS_ASSERT_MSG(0, "Target Addr not found!");
  return 0; // Or send 404
}

#endif // _ACE_MASTER_H_
