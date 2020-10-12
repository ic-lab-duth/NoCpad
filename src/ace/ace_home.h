// --------------------------------------------------------- //
//       HOME-NODE - Sorts the coherent transactions         //
// --------------------------------------------------------- //

#ifndef _ACE_HOME_H_
#define _ACE_HOME_H_

#include "systemc.h"
#include "nvhls_connections.h"

#include "../include/ace.h"
#include "../include/flit_ace.h"
#include "../include/duth_fun.h"

// --- HOME NODE ---
// All coherent transactions are serialized to a HOME NODE.
// HOME generates the apropriate Snoop requests and gathers their responses
// Regarding the responses of the snooped masters, HOME decides if access to
//   a main memory (i.e. Slave) is required for the transaction completion
template <typename cfg>
SC_MODULE(ace_home) {
  typedef typename ace::ace5<axi::cfg::ace> ace5_;
  typedef typename ace::ACE_Encoding        enc_;
  
  typedef flit_dnp<cfg::RREQ_PHITS>  rreq_flit_t;
  typedef flit_dnp<cfg::RRESP_PHITS> rresp_flit_t;
  typedef flit_dnp<cfg::WREQ_PHITS>  wreq_flit_t;
  typedef flit_dnp<cfg::WRESP_PHITS> wresp_flit_t;
  typedef flit_dnp<cfg::CREQ_PHITS>  creq_flit_t;
  typedef flit_dnp<cfg::CRESP_PHITS> cresp_flit_t;
  
  typedef flit_ack ack_flit_t;
  
  typedef sc_uint< clog2<cfg::RRESP_PHITS>::val > cnt_phit_rresp_t;
  typedef sc_uint< clog2<cfg::WRESP_PHITS>::val > cnt_phit_wresp_t;
  
  const unsigned char LOG_RD_M_LANES = nvhls::log2_ceil<cfg::RD_LANES>::val;
  const unsigned char LOG_WR_M_LANES = nvhls::log2_ceil<cfg::WR_LANES>::val;
  
  sc_in_clk    clk;
  sc_in <bool> rst_n;
  
  sc_in < sc_uint<(dnp::ace::AH_W+dnp::ace::AL_W)> > addr_map[cfg::SLAVE_NUM][2];
  
  sc_in< sc_uint<dnp::S_W> > THIS_ID;
  
  // NoC Side Channels
  Connections::Out<creq_flit_t> INIT_S1(cache_req);
  Connections::In<cresp_flit_t> INIT_S1(cache_resp);
  
  Connections::In<rreq_flit_t>   INIT_S1(rd_from_master);
  Connections::Out<rresp_flit_t> INIT_S1(rd_to_master);
  
  Connections::Out<rreq_flit_t> INIT_S1(rd_to_slave);
  Connections::In<rresp_flit_t> INIT_S1(rd_from_slave);
  
  Connections::In<wreq_flit_t> INIT_S1(wr_from_master);
  Connections::Out<wresp_flit_t> INIT_S1(wr_to_master);
  
  Connections::Out<wreq_flit_t> INIT_S1(wr_to_slave);
  Connections::In<wresp_flit_t> INIT_S1(wr_from_slave);
  
  Connections::In<ack_flit_t> INIT_S1(ack_from_master);

  
  // Constructor
  SC_HAS_PROCESS(ace_home);
  ace_home(sc_module_name name_="ace_home")
    :
    sc_module (name_)
  {
    SC_THREAD(req_check);
    sensitive << clk.pos();
    async_reset_signal_is(rst_n, false);
  }
  
  
  void req_check () {
    cache_req.Reset();
    cache_resp.Reset();
    rd_from_master.Reset();
    rd_to_master.Reset();
    rd_to_slave.Reset();
    rd_from_slave.Reset();
    
    wr_from_master.Reset();
    wr_to_master.Reset();
    wr_to_slave.Reset();
    wr_from_slave.Reset();
    
    //-- End of Reset ---//

    while(1) {
      wait();
      // Initial Request
      // Wait until a read or write request is received
      rreq_flit_t flit_req_rcv;
      bool got_new_req = false;
      do {
        if (!got_new_req) got_new_req = wr_from_master.PopNB(flit_req_rcv);
        if (!got_new_req) got_new_req = rd_from_master.PopNB(flit_req_rcv);
        wait();
      } while (!got_new_req);
      
      // Check who initiated the transaction the the type of transaction
      ace5_::AddrPayload  cur_req;
      unsigned initiator = flit_req_rcv.get_src();
      bool     is_read   = (flit_req_rcv.get_type() == dnp::PACK_TYPE__RD_REQ);
      bool     is_write  = (flit_req_rcv.get_type() == dnp::PACK_TYPE__WR_REQ);
      NVHLS_ASSERT_MSG(is_read ^ is_write , "ERROR : Home got request of wrong type.");
      flit_req_rcv.get_rd_req(cur_req);
      
      #ifndef __SYNTHESIS__
      if(is_read) std::cout << "[HOME "<< THIS_ID <<"] Got RD from " << initiator << " : " << cur_req << " @" << sc_time_stamp() << "\n";
      else        std::cout << "[HOME "<< THIS_ID <<"] Got WR from " << initiator << " : " << cur_req << " @" << sc_time_stamp() << "\n";
      #endif
      NVHLS_ASSERT_MSG(((flit_req_rcv.data[0].to_uint() >> dnp::D_PTR) & ((1<<dnp::D_W)-1)) == (THIS_ID.read().to_uint()), "Flit misrouted!");
  
      // Build the appropriate Snoop request for the cached FULL ACE Masters, depending the coherent access
      ace5_::AC snoop_req;
      snoop_req.addr  = cur_req.addr;
      snoop_req.snoop = is_read ? ace::rd_2_snoop(cur_req.snoop) : ace::wr_2_snoop(cur_req.snoop);
      snoop_req.prot  = initiator;
  
      creq_flit_t flit_snoop;
      flit_snoop.type = SINGLE; // Entire request fits in single flits thus SINGLE
      flit_snoop.set_network(THIS_ID, 0, 0, (is_read ? dnp::PACK_TYPE__RD_REQ : dnp::PACK_TYPE__WR_REQ), 0);
      flit_snoop.set_snoop_req(snoop_req);
  
      // Send the Snoop requests to the masters. This can exploit multicast capabilities of the routers
      for (int i=cfg::SLAVE_NUM; i<cfg::SLAVE_NUM+cfg::FULL_MASTER_NUM; ++i) {
        if(i!=initiator) {
          flit_snoop.set_dst(i);
          cache_req.Push(flit_snoop);
          wait();
        }
      }
      
      // Depending the initiating master (Lite or Full Ace) different number of request/repsonses are expected
      bool init_is_full  = (initiator<(cfg::SLAVE_NUM+cfg::FULL_MASTER_NUM));
      unsigned resp_wait = init_is_full ? cfg::FULL_MASTER_NUM-1 : cfg::FULL_MASTER_NUM;
      // Wait until all responses are received
      cresp_flit_t snoop_resp_data;
      ace5_::CR::Resp resp_accum = 0;
      bool got_data  = false;
      bool got_dirty = false;
      while (resp_wait) {
        cresp_flit_t flit_rcv_snoop_resp = cache_resp.Pop();
        NVHLS_ASSERT_MSG((flit_rcv_snoop_resp.type == HEAD || flit_rcv_snoop_resp.type == SINGLE), "Snoop Responce Must be at HEAD/SINGLE flit.");
        
        // Each response is checked if it contains data and accumulate the response to conclude to an action
        ace5_::CR::Resp cur_snoop_resp;
        cur_snoop_resp = (flit_rcv_snoop_resp.data[0] >> dnp::ace::cresp::C_RESP_PTR) & ((1<<dnp::ace::C_RESP_W)-1);
        bool has_data  = cur_snoop_resp & 0x1;
        bool has_dirty = cur_snoop_resp & 0x4;
        if (has_data) {
          if (has_dirty || !got_data) {
            snoop_resp_data = cache_resp.Pop();  // Get data
            NVHLS_ASSERT_MSG((snoop_resp_data.type == TAIL), "Currently a single flit cache line is supported!");
          } else {
            cresp_flit_t drop_data = cache_resp.Pop();                    // Already got Data, thus drop any other
            NVHLS_ASSERT_MSG((drop_data.type == TAIL), "Currently a single flit cache line is supported!" );
          }
        }
        resp_accum |= cur_snoop_resp;
        got_dirty  |= cur_snoop_resp & 0x4;
        got_data   |= cur_snoop_resp & 0x1;
        resp_wait--;
      }
      
      // If initiator demands clean, update Mem in case of dirty line
      bool update_mem = req_denies_dirty(cur_req.snoop, is_read);
      if (got_dirty && update_mem) {
        unsigned mem_to_write = addr_lut(cur_req.addr);
        wreq_flit_t mem_upd_flit;
        mem_upd_flit.type = HEAD;
        mem_upd_flit.data[0] = flit_req_rcv.data[0];
        mem_upd_flit.data[1] = flit_req_rcv.data[1];
        mem_upd_flit.data[2] = flit_req_rcv.data[2];
        mem_upd_flit.set_network(THIS_ID, mem_to_write, 0, dnp::PACK_TYPE__C_WR_REQ, 0);
        
        wr_to_slave.Push(mem_upd_flit);
        #pragma hls_unroll yes
        for (int i=0; i<cfg::WREQ_PHITS; ++i) {
          mem_upd_flit.data[i] = snoop_resp_data.data[i] | (((sc_uint<dnp::PHIT_W>)3) << dnp::ace::wdata::E0_PTR);
        }
        mem_upd_flit.type = TAIL;
        wr_to_slave.Push(mem_upd_flit);
        wr_from_slave.Pop(); // ToDo : Maybe error handling
        
        resp_accum = resp_accum & 0x1B; // Drop Pass Dirty bit as it got writen in Mem
      }
      
      if (is_read) {
        bool data_are_expected = req_expects_data(cur_req.snoop, is_read);
        // After responces are gathered, either respond to initiating master, or ask Main_mem/LLC
        if (data_are_expected) {
          // After responces are gathered, either respond to initiating master, or ask Main_mem/LLC
          if (!got_data) { //  Didn't get data response, thus  ask memory
            //  Didn't get data response, thus  ask memory
            unsigned mem_to_req = addr_lut(cur_req.addr);
            flit_req_rcv.set_network(THIS_ID, mem_to_req, 0, dnp::PACK_TYPE__C_RD_REQ, 0);
      
            rd_to_slave.Push(flit_req_rcv);
      
            rd_from_slave.Pop(); // Drop the header
      
            snoop_resp_data = rd_from_slave.Pop();
            resp_accum = ((snoop_resp_data.data[0] >> dnp::ace::rdata::RE_PTR) & 0x3) | (resp_accum & 0xC);
          } else {
            resp_accum = resp_accum & 0xE; // MASK WasUnique and HasData. Easily creating the R resp from CR resp
          }
          #pragma hls_unroll yes
          for (int i=0; i<cfg::RRESP_PHITS; ++i) {
            snoop_resp_data.data[i] |= snoop_resp_data.data[i] | (((sc_uint<dnp::PHIT_W>)resp_accum) << dnp::ace::rdata::RE_PTR);
          }
        } else {
          // Master does not expect Data, thus build empty data+response And write any data received to Mem
          resp_accum = resp_accum & 0xE; // MASK WasUnique and HasData. Easily creating the R resp from CR resp
          #pragma hls_unroll yes
          for (int i=0; i<cfg::RRESP_PHITS; ++i) {
            snoop_resp_data.data[i] = (((sc_uint<dnp::PHIT_W>) resp_accum) << dnp::ace::rdata::RE_PTR);
          }
          snoop_resp_data.type = TAIL;
        }
        // Build and send reponse packet
        rresp_flit_t flit_resp_to_init;
        flit_resp_to_init.type  = HEAD;
        flit_resp_to_init.set_network(THIS_ID, initiator, 0, dnp::PACK_TYPE__C_RD_RESP, 0);
        flit_resp_to_init.set_rd_resp(cur_req);
  
        rd_to_master.Push(flit_resp_to_init); // Send Header flit
        rd_to_master.Push(snoop_resp_data);   // Send Data. IsSHared and IsDirty are expected to be 0
      } else {
        // Init transaction is a Write thus resolbe Mem to write and send the Write transaction
        unsigned mem_to_write = addr_lut(cur_req.addr);
        flit_req_rcv.set_network(THIS_ID, mem_to_write, 0, dnp::PACK_TYPE__C_WR_REQ, 0);
        wr_to_slave.Push(flit_req_rcv); // Send Head
        
        wreq_flit_t data_to_update;
        while(!wr_from_master.PopNB(data_to_update)) {
          wait();
        }
        wr_to_slave.Push(data_to_update);
  
        // transfer the response to the init Master
        wresp_flit_t mv_wr_resp = wr_from_slave.Pop();
        mv_wr_resp.set_src(THIS_ID); // Set the HOME src to receive the response
        mv_wr_resp.set_dst(initiator); // Set the initiator as a recipient  src to receive the response
        wr_to_master.Push(mv_wr_resp);
      }
      
      // Wait for the final ack from the Init Master that signifies that the cache has been completed the transaction
      // Only a Full Master responds with an Ack
      if (init_is_full) {
        ack_flit_t rcv_ack = ack_from_master.Pop();
        NVHLS_ASSERT_MSG( (rcv_ack.is_rack()&&is_read) || (rcv_ack.is_wack()&&(!is_read)), "ACK does not match the responce (i.e. RD/WR)");

        #ifndef __SYNTHESIS__
        if(is_read) std::cout << "[HOME "<< THIS_ID <<"] Got RD ACK from " << rcv_ack.get_src() << " : " << cur_req << " @" << sc_time_stamp() << "\n";
        else        std::cout << "[HOME "<< THIS_ID <<"] Got WR AK  from " << rcv_ack.get_src() << " : " << cur_req << " @" << sc_time_stamp() << "\n";
        #endif
      }
      
      
    } // End of while(1)
  }; // End of HOME Node
  
  // Transactions that do not accept Dirty data, thus the interconnects is responsible for to handle them
  inline bool req_denies_dirty(NVUINTW(enc_::ARSNOOP::_WIDTH) &request_in, bool is_read ) {
    if (is_read) {
      if ((request_in == enc_::ARSNOOP::RD_ONCE) ||
          (request_in == enc_::ARSNOOP::RD_CLEAN) ||
          (request_in == enc_::ARSNOOP::RD_NOT_SHARED_DIRTY) ||
          (request_in == enc_::ARSNOOP::CLEAN_UNIQUE) ||
          (request_in == enc_::ARSNOOP::MAKE_UNIQUE)  ||
          (request_in == enc_::ARSNOOP::CLEAN_SHARED)  ||
          (request_in == enc_::ARSNOOP::CLEAN_INVALID)  ||
          (request_in == enc_::ARSNOOP::MAKE_INVALID)  )
      {
        return true;
      }
    } else { // request is a write, thus any dirty must be sent to Mem
      return true;
    }
    return false;
  };
    
    // When data are required, HOME must access Mem when all snoops where missed
  inline bool req_expects_data(NVUINTW(enc_::ARSNOOP::_WIDTH) &request_in, bool is_read ) {
    if (is_read) {
      if ((request_in == enc_::ARSNOOP::RD_ONCE) ||
          (request_in == enc_::ARSNOOP::RD_CLEAN) ||
          (request_in == enc_::ARSNOOP::RD_NOT_SHARED_DIRTY) ||
          (request_in == enc_::ARSNOOP::RD_SHARED) ||
          (request_in == enc_::ARSNOOP::RD_UNIQUE)  )
      {
        return true;
      }
    }
    return false;
  };
  
  // Memory map resolving 
  inline unsigned char addr_lut(const ace5_::Addr addr) {
    for (int i=0; i<2; ++i) {
      if (addr>=addr_map[i][0].read() && addr <= addr_map[i][1].read()) return i;
    }
    return 0; // Or send 404
  };
}; // End of Home module

#endif // _ACE_HOME_H_
