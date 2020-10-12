#ifndef AXI_COHERENCY_CHECKER_H
#define AXI_COHERENCY_CHECKER_H

#include "systemc.h"

#include "../../src/include/dnp_ace_v0.h"

#include <deque>
#include <queue>

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int FULL_MASTER_NUM, unsigned int LITE_MASTER_NUM, unsigned int SLAVE_NUM>
SC_MODULE(ace_coherency_checker) {
  typedef typename ace::ace5<axi::cfg::ace> ace5_;
  typedef typename ace::ACE_Encoding        enc_;
  
	sc_in_clk    clk;
  sc_in <bool> rst_n;
	
	sc_in<bool>  stop_gen; // Not Used
  
  sc_in< sc_uint<32> >  addr_map[SLAVE_NUM][2];
  
  // Scoreboard
  sc_mutex                                                      *sb_lock;
  std::vector< std::deque< msg_tb_wrap<ace5_::AddrPayload> > >  *sb_rd_req_q;
  std::vector< std::deque< msg_tb_wrap<ace5_::ReadPayload> > >  *sb_rd_resp_q;
  
  std::vector< std::deque< msg_tb_wrap<ace5_::AddrPayload> > >   *sb_wr_req_q;
  std::vector< std::deque< msg_tb_wrap<ace5_::WritePayload> > >  *sb_wr_data_q;
  std::vector< std::deque< msg_tb_wrap<ace5_::WRespPayload> > >  *sb_wr_resp_q;
  
  std::vector< std::deque< msg_tb_wrap<ace5_::AddrPayload> > >   *sb_coherent_access_q;
  std::vector< std::deque< msg_tb_wrap<ace5_::AC> > >            *sb_snoop_req_q;
  std::vector< std::deque< msg_tb_wrap<ace5_::CR> > >            *sb_snoop_resp_q;
  std::vector< std::deque< msg_tb_wrap<ace5_::CD> > >            *sb_snoop_data_resp_q;
  
	unsigned int total_cycles;
  
  sc_time clk_period;
  
  class snoop_set {
  public:
    ace5_::AC ac;
    ace5_::CR cr;
    ace5_::CD cd;
  };
  
  class snoop_trans_bundle {
  public:
    //snoop_set snoops[MASTER_NUM-1];
    ace5_::AC ac[FULL_MASTER_NUM];
    ace5_::CR cr[FULL_MASTER_NUM];
    ace5_::CD cd[FULL_MASTER_NUM];
    bool      valid[FULL_MASTER_NUM];
    unsigned  count; // Must never
    
    snoop_trans_bundle () {
      for(int i=0; i<FULL_MASTER_NUM; ++i) valid[i] = false;
      count = 0;
    };
    
    snoop_trans_bundle (
      unsigned master_id_,
      ace5_::AC ac_,
      ace5_::CR cr_,
      ace5_::CD cd_
    ) {
      ac[master_id_] = ac_;
      cr[master_id_] = cr_;
      cd[master_id_] = cd_;
      for(int i=0; i<FULL_MASTER_NUM; ++i) valid[i] = (i == master_id_);
      count = 1;
    };
    
    unsigned push (
            unsigned master_id_,
            ace5_::AC ac_,
            ace5_::CR cr_,
            ace5_::CD cd_
    ) {
      // Error handling ifs
      if (count >= (FULL_MASTER_NUM)) {
        std::cout << "Got more snoops than expected for addr: " << ac_.addr << "\n";
        NVHLS_ASSERT(0);
      } else if (valid[master_id_]) {
        std::cout << "Second snoop at M#" << master_id_ << " for addr: " << std::hex << ac_.addr << std::dec << "\n";
        NVHLS_ASSERT(0);
      }
      
      ac[master_id_]    = ac_;
      cr[master_id_]    = cr_;
      cd[master_id_]    = cd_;
      valid[master_id_] = true;
      count++;
      return count;
    };
    
    void clear () {
      for(int i=0; i<FULL_MASTER_NUM; ++i) valid[i] = false;
      count = 0;
    };
  };
  
  // Vars to conclude who is the initiator of the responses
  // -- We know that we expect N-1 snoop reqs/resps and the not received is the initiator, thus at most 2 addresses could be in front
  // -- Currently we expect all requests in order, thus the snoop requests are going to be at the front of
  //    the queue. In case of multiple HOME nodes the above assumtions break
  // -- each request/response is identifiable by its address and requests of the same address MUST be in-order
  //    This info should be used, to support multiple Home nodes, aka search in the queues for the first address occurrence
  
  // Now each address, has a struct that collects snoops from the various target masters.
  // At no point should exist two requests for the same address at the same target.
  // -- In case more requests for the same address is allowed the internal class should provide distinct queues for each target
  //    which will facilitate the requests that MUST be in order.
  // -- The above should probably never happen since to concurrent snoop requests are indistinguishable,
  //    thus it's impossible for initiators to sort the sequence and transient states would be necessary.
  
  std::map<ace5_::Addr, snoop_trans_bundle> scrutineer; // Wow, nice word. Not gonna remember it. Don't care. Still Gonna use. Long live autocomplete!
  
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
  
  void     manage_bundle   (snoop_trans_bundle & cur_trans_bundle, unsigned initiator);
  unsigned mem_map_resolve (ace5_::Addr &addr);
  bool     req_denies_dirty (NVUINTW(enc_::ARSNOOP::_WIDTH) &request_in, bool is_read );
  bool     req_no_data_resp (NVUINTW(enc_::ARSNOOP::_WIDTH) &request_in, bool is_read );
  
  // Constructor
  SC_HAS_PROCESS(ace_coherency_checker);
  ace_coherency_checker(sc_module_name name_="ace_coherency_checker") : sc_module(name_)
  {
		SC_THREAD(do_cycle);
    sensitive << clk.pos();
    reset_signal_is(rst_n, false);
  }
};


// --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- //
// --- --- --- --- --- --- IMPLEMENTATION  --- --- --- --- --- --- --- --- //
// --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- //

template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int FULL_MASTER_NUM, unsigned int LITE_MASTER_NUM, unsigned int SLAVE_NUM>
void ace_coherency_checker<RD_M_LANES, RD_S_LANES, WR_M_LANES, WR_S_LANES, FULL_MASTER_NUM, LITE_MASTER_NUM, SLAVE_NUM>::do_cycle () {
  total_cycles = 0;
  clk_period   = (dynamic_cast<sc_clock *>(clk.get_interface()))->period();
  
  ace5_::AC snoop_req;
  ace5_::CR snoop_resp;
  ace5_::CD snoop_data;
  
  while(1) {
    wait();
    
    sb_lock->lock();
    for (unsigned i=0; i<FULL_MASTER_NUM; ++i) {
      if ( !(*sb_snoop_resp_q)[i].empty() ) {
        
        snoop_req  = (*sb_snoop_req_q)[i].front().dut_msg;
        (*sb_snoop_req_q)[i].pop_front();
        snoop_resp = (*sb_snoop_resp_q)[i].front().dut_msg;
        (*sb_snoop_resp_q)[i].pop_front();
        if (snoop_resp.resp & 1) {
          snoop_data = (*sb_snoop_data_resp_q)[i].front().dut_msg;
          (*sb_snoop_data_resp_q)[i].pop_front();
        }
  
        // resolve who is the initiator, to know the number of the expected snoop reqs
        // ID format -> 0 - SLAVES - FULL_MASTERS - LITE_MASTERS - HOMES
        unsigned initiator = snoop_req.prot;
        NVHLS_ASSERT_MSG(initiator>=SLAVE_NUM, "A SLAVE cannot be an initiator. (I.e. ID<SLAVE_NUM)")
        NVHLS_ASSERT_MSG(initiator<(SLAVE_NUM+FULL_MASTER_NUM+LITE_MASTER_NUM), "A HOME cannot be an initiator. (I.e. ID greater than the masters')")
        unsigned init_is_ace  = (initiator<(SLAVE_NUM+FULL_MASTER_NUM));
        unsigned init_is_lite = (initiator>=(SLAVE_NUM+FULL_MASTER_NUM));
        NVHLS_ASSERT_MSG(initiator<(SLAVE_NUM+FULL_MASTER_NUM+LITE_MASTER_NUM), "Wot?!?! Check the 2lines above....")
        
        
        
        auto cur_trans_bundle_it = scrutineer.find(snoop_req.addr);
        //if tha address is not init in scrutineer, create it.
        if ( cur_trans_bundle_it == scrutineer.end() ) { // Not found
          //scrutineer[snoop_req.addr] = snoop_trans_bundle(i, snoop_req, snoop_resp, snoop_data);
          scrutineer[snoop_req.addr] = snoop_trans_bundle();
          cur_trans_bundle_it = scrutineer.find(snoop_req.addr);
        }
        
        
        unsigned new_count = (cur_trans_bundle_it->second).push(i, snoop_req, snoop_resp, snoop_data);
        // Existing Address of scrutineer
        // Bundled completed, Set next expected packets and reset it.
        unsigned master_initiator = (initiator - SLAVE_NUM);
        if ( init_is_ace && (new_count == FULL_MASTER_NUM-1) ) {
          manage_bundle(cur_trans_bundle_it->second, master_initiator);
          (cur_trans_bundle_it->second).clear();
          //scrutineer.erase(cur_trans_bundle_it); // ToDo : consider completely remove it to save space/ but lose time constr/deconstr
        } else if (init_is_lite && (new_count == FULL_MASTER_NUM) ) {
          manage_bundle(cur_trans_bundle_it->second, master_initiator);
          (cur_trans_bundle_it->second).clear();
        }
      } // End of new bundle handle
    } // End for Masters
    sb_lock->unlock();
    
  } // End of while(1)
}; // End of do_cycle





// --------------------------- //
// --- GENERATOR Functions --- //
// --------------------------- //

template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int FULL_MASTER_NUM, unsigned int LITE_MASTER_NUM, unsigned int SLAVE_NUM>
void ace_coherency_checker<RD_M_LANES, RD_S_LANES, WR_M_LANES, WR_S_LANES, FULL_MASTER_NUM, LITE_MASTER_NUM, SLAVE_NUM>::manage_bundle (snoop_trans_bundle & cur_trans_bundle, unsigned initiator) {
  
  // This is deprecated, when the lack of a snoop response gives hints the initiator.
  // To add ACE LITE nodes, this is impossible and the initiator is passed by the HOME through the PROT field
  // Find the initiator
  /*
  unsigned initiator = MASTER_NUM+1;
  for (int i=0; i<MASTER_NUM; ++i) {
    if (!cur_trans_bundle.valid[i]) {
      NVHLS_ASSERT_MSG(initiator>MASTER_NUM, "Bundle went wrong. There are two masters that haven't received Snoop reqs.");
      initiator = i;
    }
  }
  NVHLS_ASSERT_MSG(initiator<MASTER_NUM, "Initiator not found!");
  */
  
  // Find the initiating request from the Scoreboard
  ace5_::AddrPayload coherent_init;
  bool               coherent_init_found = false;
  bool is_read;
  int dbg_size = (*sb_coherent_access_q)[initiator].size();
  for (int i=0; i<(*sb_coherent_access_q)[initiator].size(); ++i) {
    msg_tb_wrap<ace5_::AddrPayload> cur_coherent_req = ((*sb_coherent_access_q)[initiator][i]);
    if (cur_coherent_req.dut_msg.addr == cur_trans_bundle.ac[(initiator+1)%FULL_MASTER_NUM].addr) {
      coherent_init       = cur_coherent_req.dut_msg;
      coherent_init_found = true;
      is_read             = cur_coherent_req.is_read;
      (*sb_coherent_access_q)[initiator].erase((*sb_coherent_access_q)[initiator].begin()+i);
      break;
    }
  }
  NVHLS_ASSERT_MSG(coherent_init_found, "Init transaction not found!");
  
  // Check that all Snoop requests are the same and of correct type.
  ace5_::AC snoop_type_expected;
  snoop_type_expected.addr  = coherent_init.addr;
  snoop_type_expected.snoop = is_read ? ace::rd_2_snoop(coherent_init.snoop) : ace::wr_2_snoop(coherent_init.snoop);
  for (int i=0; i<FULL_MASTER_NUM; ++i) {
    if(i != initiator) {
      bool addr_is_equal  = (cur_trans_bundle.ac[i].addr == snoop_type_expected.addr);
      bool snoop_is_equal = (cur_trans_bundle.ac[i].snoop == snoop_type_expected.snoop);
      if (!(addr_is_equal && snoop_is_equal)) {
        std::cout << "Home node didn't send correct snoop requests for access: " << coherent_init << "\n";
        std::cout << "  - Expected:  " << snoop_type_expected << "\n";
        std::cout << "  - Got @[MASTER" << i+SLAVE_NUM << "]:  " << cur_trans_bundle.ac[i] << "\n";
        NVHLS_ASSERT(0);
      }
    }
  }
  
  // go through responses to see if there are data available
  ace5_::CD       snoop_data;
  ace5_::CR::Resp resp_accum = 0;
  bool got_data  = false;
  bool got_dirty = false;
  for (int i=0; i<FULL_MASTER_NUM; ++i) {
    if(i != initiator) {
      resp_accum |= cur_trans_bundle.cr[i].resp;
      bool this_has_data = cur_trans_bundle.cr[i].resp & 0x1;
      bool this_dirty    = cur_trans_bundle.cr[i].resp & 0x4;
  
      if(got_data && this_has_data) {
        if (snoop_data.data != cur_trans_bundle.cd[i].data){
          std::cout << "ERR : Caches have different values for valid cache lines!\n";
          std::cout << "      " << snoop_data << "\n";
          std::cout << "      " << cur_trans_bundle.cd[i] << "\n";
          NVHLS_ASSERT(0);
        }
      }
      if (got_dirty && this_dirty) {
        std::cout << "ERR : Got 2 Dirty lines for Addr: " << snoop_type_expected.addr << "\n";
        NVHLS_ASSERT(0);
      }
  
      if (this_dirty || (this_has_data && !got_data)) {
        got_data   = true;
        got_dirty  = this_dirty;
        snoop_data = cur_trans_bundle.cd[i];
      }
    }
  }
  
  // When Dirty gets Written back to Mem
  bool req_no_dirty = req_denies_dirty(coherent_init.snoop, is_read) || req_no_data_resp(coherent_init.snoop, is_read);
  // When dirty resp to req that denies dirty, writeback data to Mem
  if (got_dirty && req_no_dirty) {
    // Setup scoreboards to reflect the responses
    msg_tb_wrap<ace5_::AddrPayload> temp_wr_req_tb;
    temp_wr_req_tb.dut_msg = coherent_init;
    //Mask all ACE fields
    temp_wr_req_tb.dut_msg.snoop   = 0;
    temp_wr_req_tb.dut_msg.domain  = 0;
    temp_wr_req_tb.dut_msg.barrier = 0;
    
    unsigned tgt_mem = mem_map_resolve(coherent_init.addr);
    (*sb_wr_req_q)[tgt_mem].push_back(temp_wr_req_tb);
    
    msg_tb_wrap< ace5_::WritePayload > wr_back_beat;
    wr_back_beat.dut_msg.data  = snoop_data.data;
    wr_back_beat.dut_msg.last  = snoop_data.last;
    wr_back_beat.dut_msg.wstrb = -1;
    (*sb_wr_data_q)[tgt_mem].push_back(wr_back_beat);
  }
  
  // Setup scoreboards to reflect the responses
  if (is_read) { // Read Coherent access
    // If there are no data, HOME will access Memory
    bool req_no_data = req_no_data_resp(coherent_init.snoop, is_read);
    if ( got_data || req_no_data ) {
      // Push Read Response to expect at initiator
      ace5_::ReadPayload data_responce;
      data_responce.data = req_no_data ? (ace5_::CD::Data) 0 : snoop_data.data;
      data_responce.last = req_no_data ? (ace5_::CD::Last) 1 : snoop_data.last;
      data_responce.resp = req_no_dirty ? resp_accum & 0xA : resp_accum & 0xE; // If request denies Dirty, Expect dropped PassDirty
      data_responce.id   = coherent_init.id;
  
      msg_tb_wrap<ace5_::ReadPayload> temp_rd_resp_tb;
      temp_rd_resp_tb.dut_msg = data_responce;
  
      (*sb_rd_resp_q)[initiator].push_back(temp_rd_resp_tb);
      
    } else {
      // Generate expected request to Mem
      msg_tb_wrap<ace5_::AddrPayload> temp_rd_req_tb;
      temp_rd_req_tb.dut_msg = coherent_init;
      //Mask all ACE fields
      temp_rd_req_tb.dut_msg.snoop   = 0;
      temp_rd_req_tb.dut_msg.domain  = 0;
      temp_rd_req_tb.dut_msg.barrier = 0;
      
      unsigned tgt_mem = mem_map_resolve(coherent_init.addr);
      (*sb_rd_req_q)[tgt_mem].push_back(temp_rd_req_tb);
      
      // Generate expected responce from Mem
      ace5_::ReadPayload beat_expected;
      beat_expected.data = 0;
      beat_expected.id   = coherent_init.id;
      unsigned byte_count = 0;
      unsigned bytes_total = (ace5_::C_CACHE_WIDTH/8);
      while(byte_count<bytes_total) {
        beat_expected.data |= ( ((ace5_::Data)(byte_count & 0xFF)) << ((ace5_::Data)(byte_count*8)));
        byte_count++;
    
        if(((byte_count%(ace5_::C_DATA_CHAN_WIDTH/8))==0) || (byte_count == bytes_total)) {
          beat_expected.resp = tgt_mem;
          beat_expected.last = (byte_count == bytes_total);
          
          msg_tb_wrap< ace5_::ReadPayload > temp_rd_resp_tb;
          temp_rd_resp_tb.dut_msg  = beat_expected;
          temp_rd_resp_tb.time_gen = sc_time_stamp();
      
          (*sb_rd_resp_q)[initiator].push_back(temp_rd_resp_tb);
          beat_expected.data = 0;
        }
      }
    }
  } else { // Write Coherent access
    // The expected behavior is handled by the generator, being unchanged.
  }
};


// ------------------------ //
// --- VERIFY Functions --- //
// ------------------------ //

template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int FULL_MASTER_NUM, unsigned int LITE_MASTER_NUM, unsigned int SLAVE_NUM>
bool ace_coherency_checker<RD_M_LANES, RD_S_LANES, WR_M_LANES, WR_S_LANES, FULL_MASTER_NUM, LITE_MASTER_NUM, SLAVE_NUM>::req_denies_dirty (NVUINTW(enc_::ARSNOOP::_WIDTH) &request_in, bool is_read ) {
  if (is_read) {
    if ((request_in == enc_::ARSNOOP::RD_ONCE) ||
        (request_in == enc_::ARSNOOP::RD_CLEAN) ||
        (request_in == enc_::ARSNOOP::RD_NOT_SHARED_DIRTY) ||
        (request_in == enc_::ARSNOOP::CLEAN_UNIQUE) ||
        (request_in == enc_::ARSNOOP::CLEAN_INVALID))
    {
      return true;
    }
  } else {
    return true;
  }
  return false;
};

template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int FULL_MASTER_NUM, unsigned int LITE_MASTER_NUM, unsigned int SLAVE_NUM>
bool ace_coherency_checker<RD_M_LANES, RD_S_LANES, WR_M_LANES, WR_S_LANES, FULL_MASTER_NUM, LITE_MASTER_NUM, SLAVE_NUM>::req_no_data_resp (NVUINTW(enc_::ARSNOOP::_WIDTH) &request_in, bool is_read ) {
  if (is_read) {
    if ((request_in == enc_::ARSNOOP::CLEAN_UNIQUE) ||
        (request_in == enc_::ARSNOOP::MAKE_UNIQUE) ||
        (request_in == enc_::ARSNOOP::CLEAN_SHARED) ||
        (request_in == enc_::ARSNOOP::CLEAN_INVALID) ||
        (request_in == enc_::ARSNOOP::MAKE_INVALID) )
    {
      return true;
    }
  }
  return false;
};



template <unsigned int RD_M_LANES, unsigned int RD_S_LANES, unsigned int WR_M_LANES, unsigned int WR_S_LANES, unsigned int FULL_MASTER_NUM, unsigned int LITE_MASTER_NUM, unsigned int SLAVE_NUM>
unsigned ace_coherency_checker<RD_M_LANES, RD_S_LANES, WR_M_LANES, WR_S_LANES, FULL_MASTER_NUM, LITE_MASTER_NUM, SLAVE_NUM>::mem_map_resolve (ace5_::Addr &addr) {
  for (int i=0; i<SLAVE_NUM; ++i) {
    if (addr>=addr_map[i][0].read() && addr <= addr_map[i][1].read()) return i;
  }
  NVHLS_ASSERT_MSG(0, "Target Addr not found!");
  return 0; // Or send 404
}

#endif // AXI_COHERENCY_CHECKER_H
