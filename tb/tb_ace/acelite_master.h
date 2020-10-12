#ifndef _ACE_LITE_MASTER_H_
#define _ACE_LITE_MASTER_H_

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
SC_MODULE(acelite_master) {
  typedef typename ace::ace5<axi::cfg::ace> ace5_;
  typedef typename ace::ACE_Encoding        enc_;
  
	sc_in_clk    clk;
  sc_in <bool> rst_n;

	sc_in<bool>  stop_gen;
 
	sc_in< sc_uint<32> >  addr_map[SLAVE_NUM][2];
  
  // Master ACE Ports
  
  Connections::Out<ace5_::AddrPayload>   ar_out;
  Connections::In<ace5_::ReadPayload>    r_in;
  
  Connections::Out<ace5_::AddrPayload>   aw_out;
  Connections::Out<ace5_::WritePayload>  w_out;
  Connections::In<ace5_::WRespPayload>   b_in;
  
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
  
  std::deque<ace5_::AddrPayload>  sb_rd_order_q; // queue to check ordering
  std::deque<ace5_::AddrPayload>  sb_wr_order_q; // queue to check ordering
  
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
 
	void verify_rd_resp(ace5_::ReadPayload  &rcv_rd_resp);
	void verify_wr_resp(ace5_::WRespPayload &rcv_wr_resp);
  
  bool eq_rd_data(ace5_::ReadPayload &rcv_rd_data, ace5_::ReadPayload &sb_rd_data);
  bool eq_wr_resp(ace5_::WRespPayload &rcv_wr_resp, ace5_::WRespPayload &sb_wr_resp);
	
	unsigned mem_map_resolve(ace5_::Addr &addr);
  
  // Constructor
  SC_HAS_PROCESS(acelite_master);
    acelite_master(sc_module_name name_) : sc_module(name_)
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
void acelite_master<RD_M_LANES, RD_S_LANES, WR_M_LANES, WR_S_LANES, MASTER_NUM, SLAVE_NUM>::do_cycle () {
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
  
  ar_out.Reset();
  r_in.Reset();
  
  aw_out.Reset();
  w_out.Reset();
  b_in.Reset();
  
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
    
  }; // End of while(1)
}; // End of do_cycle

// --------------------------- //
// --- GENERATOR Functions --- //
// --------------------------- //
template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int MASTER_NUM, unsigned int SLAVE_NUM>
void acelite_master<RD_M_LANES, RD_S_LANES, WR_M_LANES, WR_S_LANES, MASTER_NUM, SLAVE_NUM>::gen_new_rd_trans() {
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
void acelite_master<RD_M_LANES, RD_S_LANES, WR_M_LANES, WR_S_LANES, MASTER_NUM, SLAVE_NUM>::gen_new_wr_trans() {
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
void acelite_master<RD_M_LANES, RD_S_LANES, WR_M_LANES, WR_S_LANES, MASTER_NUM, SLAVE_NUM>::gen_new_cache_trans() {
  ace5_::AddrPayload cache_req;
  
  cache_req.id    = MASTER_ID;// (rand() % AXI_TID_NUM); // (rand()% 2)+2;
  cache_req.size  = nvhls::log2_ceil<RD_M_LANES>::val;
  cache_req.burst = enc_::AXBURST::INCR;
  cache_req.len   = 0;
  cache_req.addr  = ((rand()%ACE_CACHE_LINES)+1) * (ace5_::C_CACHE_WIDTH>>3);//0x8;
  
  cache_req.domain = enc_::AxDOMAIN::OUTER_SHARE;
  // RD_ONCE, RD_SHARED, RD_CLEAN, RD_NOT_SHARED_DIRTY, RD_UNIQUE, CLEAN_UNIQUE, MAKE_UNIQUE, CLEAN_SHARED, CLEAN_INVALID, MAKE_INVALID
  // WR_UNIQUE, WR_LINE_UNIQUE
  bool is_read = false;
  unsigned sel_req = rand() % 6;
  if      (sel_req == 0) cache_req.snoop = enc_::ARSNOOP::RD_ONCE;
  else if (sel_req == 1) cache_req.snoop = enc_::ARSNOOP::CLEAN_SHARED;
  else if (sel_req == 2) cache_req.snoop = enc_::ARSNOOP::CLEAN_INVALID;
  else if (sel_req == 3) cache_req.snoop = enc_::ARSNOOP::MAKE_INVALID;
  else if (sel_req == 4) cache_req.snoop = enc_::AWSNOOP::WR_UNIQUE;
  else if (sel_req == 5) cache_req.snoop = enc_::AWSNOOP::WR_LINE_UNIQUE;
  is_read = (sel_req < 4);
  
  
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

// ------------------------ //
// --- VERIFY Functions --- //
// ------------------------ //
template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int MASTER_NUM, unsigned int SLAVE_NUM>
void acelite_master<RD_M_LANES, RD_S_LANES, WR_M_LANES, WR_S_LANES, MASTER_NUM, SLAVE_NUM>::verify_rd_resp(ace5_::ReadPayload &rcv_rd_resp){
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
      cache_outstanding[sb_ord_req.addr]--;
    } else {
      unsigned dbg_non_coh = 0;
    }
    rd_resp_ej++;
  }
  std::cout.flush();
  sb_lock->unlock();
}; // End of READ Response Verify

template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int MASTER_NUM, unsigned int SLAVE_NUM>
void acelite_master<RD_M_LANES, RD_S_LANES, WR_M_LANES, WR_S_LANES, MASTER_NUM, SLAVE_NUM>::verify_wr_resp(ace5_::WRespPayload &rcv_wr_resp){
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
      //upd_cache_write(sb_ord_req, rcv_wr_resp);
      cache_outstanding_writes[sb_ord_req.addr]--;
      cache_outstanding[sb_ord_req.addr]--;
    } else {
      unsigned dbg_non_coh = 0;
    }
    wr_resp_ej++;
  }
  std::cout.flush();
  sb_lock->unlock();
}; // End of WRITE Response Verify


template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int MASTER_NUM, unsigned int SLAVE_NUM>
bool acelite_master<RD_M_LANES, RD_S_LANES, WR_M_LANES, WR_S_LANES, MASTER_NUM, SLAVE_NUM>::eq_rd_data (ace5_::ReadPayload &rcv_rd_data, ace5_::ReadPayload &sb_rd_data) {
  unsigned tid_mask = (1<<dnp::ace::ID_W)-1;
  return ((rcv_rd_data.id & tid_mask) == (sb_rd_data.id & tid_mask) &&
           rcv_rd_data.data  == sb_rd_data.data  &&
           rcv_rd_data.resp  == sb_rd_data.resp  &&
           rcv_rd_data.last  == sb_rd_data.last  //&&
           //rcv_rd_data.ruser == sb_rd_data.ruser
  );
};

template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int MASTER_NUM, unsigned int SLAVE_NUM>
bool acelite_master<RD_M_LANES, RD_S_LANES, WR_M_LANES, WR_S_LANES, MASTER_NUM, SLAVE_NUM>::eq_wr_resp (ace5_::WRespPayload &rcv_wr_resp, ace5_::WRespPayload &sb_wr_resp) {
  unsigned tid_mask = (1<<dnp::ace::ID_W)-1;
  return ((rcv_wr_resp.id & tid_mask) == (sb_wr_resp.id & tid_mask) &&
           rcv_wr_resp.resp  == sb_wr_resp.resp  //&&
           //rcv_wr_resp.buser == sb_wr_resp.buser
  );
};

template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int MASTER_NUM, unsigned int SLAVE_NUM>
unsigned acelite_master<RD_M_LANES, RD_S_LANES, WR_M_LANES, WR_S_LANES, MASTER_NUM, SLAVE_NUM>::mem_map_resolve(ace5_::Addr &addr){
  for (int i=0; i<SLAVE_NUM; ++i) {
    if (addr>=addr_map[i][0].read() && addr <= addr_map[i][1].read()) return i;
  }
  NVHLS_ASSERT_MSG(0, "Target Addr not found!");
  return 0; // Or send 404
}

#endif // _ACE_LITE_MASTER_H_
