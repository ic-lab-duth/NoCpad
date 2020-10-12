// --------------------------------------------------------- //
//       MASTER-IF Is where the MASTER CONNECTS!!!!!         //
//                                                           //
// Aka. Master <-> Master-IF <-> NoC <-> Slave-IF <-> Slave  //
// --------------------------------------------------------- //


#ifndef _ACE_MASTER_IF_H_
#define _ACE_MASTER_IF_H_

#include "systemc.h"
#include "nvhls_connections.h"

#include "../include/ace.h"
#include "../include/axi4_configs_extra.h"
#include "../include/flit_ace.h"
#include "../include/duth_fun.h"

#define LOG_MAX_OUTS 8

#define INIT_S1(n)   n{#n}

// --- Helping Data structures --- //
struct outs_table_entry {
  sc_uint<dnp::D_W>     dst_last;
  sc_uint<LOG_MAX_OUTS> sent;
  bool                  reorder;
};

// Info passed between packetizer and depacketizer to inform about new and finished transactions.
struct order_info {
  sc_uint<dnp::ace::ID_W> tid;
  sc_uint<dnp::D_W>  dst;
  //bool          reord;
  //unsigned char ticket;
  
  inline friend std::ostream& operator << ( std::ostream& os, const order_info& info ) {
    os <<"TID: "<< info.tid <<", Dst: "<< info.dst /*<<", Ticket: "<< info.ticket*/;
#ifdef SYSTEMC_INCLUDED
    os << std::dec << " @" << sc_time_stamp();
#else
    os << std::dec << " @" << "no-timed";
#endif
    return os;
  }

#ifdef SYSTEMC_INCLUDED
  // Only for SystemC
  inline friend void sc_trace(sc_trace_file* tf, const order_info& info, const std::string& name) {
    sc_trace(tf, info.tid,    name + ".tid");
    sc_trace(tf, info.dst,    name + ".dst");
    //sc_trace(tf, info.ticket, name + ".ticket");
  }
#endif
};

// --- Master IF --- //
// AXI Master connects the independent AXI RD and WR cahnnels to the interface
// The interface gets the Requests and independently packetize and send them into the network
// The Responses are getting depacketized into a seperate thread and are fed back to the MASTER
// Thus Master interface comprises of 4 distinct/paarallel blocks WR/RD pack and WR/RD depack
template <typename cfg>
SC_MODULE(ace_master_if) {
  typedef typename ace::ace5<axi::cfg::ace> ace5_;
  typedef typename axi::AXI4_Encoding       enc_;
  
  typedef flit_dnp<cfg::RREQ_PHITS>  rreq_flit_t;
  typedef flit_dnp<cfg::RRESP_PHITS> rresp_flit_t;
  typedef flit_dnp<cfg::WREQ_PHITS>  wreq_flit_t;
  typedef flit_dnp<cfg::WRESP_PHITS> wresp_flit_t;
  typedef flit_dnp<cfg::CREQ_PHITS>  creq_flit_t;
  typedef flit_dnp<cfg::CRESP_PHITS> cresp_flit_t;
  
  typedef flit_ack ack_flit_t;
  
  typedef sc_uint< nvhls::log2_ceil<cfg::RRESP_PHITS>::val > cnt_phit_rresp_t;
  typedef sc_uint< nvhls::log2_ceil<cfg::WREQ_PHITS>::val >  cnt_phit_wreq_t;
  
  const unsigned char LOG_RD_M_LANES = nvhls::log2_ceil<cfg::RD_LANES>::val;
  const unsigned char LOG_WR_M_LANES = nvhls::log2_ceil<cfg::WR_LANES>::val;
  
  sc_in_clk    clk;
  sc_in <bool> rst_n;
  
  sc_in < sc_uint<(dnp::ace::AH_W+dnp::ace::AL_W)> > addr_map[cfg::SLAVE_NUM][2];
  
  sc_in< sc_uint<dnp::S_W> > THIS_ID;
  
  // AXI MASTER Side Channels
  // --- ACE --- //
  Connections::Out<ace5_::AC>  INIT_S1(ac_out);
  Connections::In <ace5_::CR>  INIT_S1(cr_in);
  Connections::In <ace5_::CD>  INIT_S1(cd_in);
  
  // --- READ --- //
  Connections::In<ace5_::AddrPayload>    INIT_S1(ar_in);
  Connections::Out<ace5_::ReadPayload>   INIT_S1(r_out);
  Connections::In <ace5_::RACK>          INIT_S1(rack_in);
  // --- WRITE --- //
  Connections::In<ace5_::AddrPayload>    INIT_S1(aw_in);
  Connections::In<ace5_::WritePayload>   INIT_S1(w_in);
  Connections::Out<ace5_::WRespPayload>  INIT_S1(b_out);
  Connections::In <ace5_::WACK>          INIT_S1(wack_in);
  
  // NoC Side Channels
  Connections::Out<rreq_flit_t> INIT_S1(rd_flit_out);
  Connections::In<rresp_flit_t> INIT_S1(rd_flit_in);
  Connections::Out<ack_flit_t>  INIT_S1(rack_flit_out);
  
  Connections::Out<wreq_flit_t> INIT_S1(wr_flit_out);
  Connections::In<wresp_flit_t> INIT_S1(wr_flit_in);
  Connections::Out<ack_flit_t>  INIT_S1(wack_flit_out);
  
  Connections::In<creq_flit_t>   INIT_S1(cache_flit_in);
  Connections::Out<cresp_flit_t> INIT_S1(cache_flit_out);
  
  
  // --- READ Internals --- //
  // FIFOs that pass initiation and finish transactions between Pack-Depack
  sc_fifo<order_info>              INIT_S1(rd_trans_init);  // Pack    to Pack     | fwd to bck
  sc_fifo<sc_uint<dnp::ace::ID_W>> INIT_S1(rd_trans_fin);   // De-pack to Pack     | bck to fwd
  
  // Placed on READ Packetizer
  outs_table_entry    rd_out_table[1<<dnp::ace::ID_W]; //[(1<<TID_W)];       // Holds the OutStanding Transactions | Hint : TID_W=4 => 16 slots x 16bits
  
  // --- WRITE Internals --- //
  sc_fifo<sc_uint<dnp::ace::ID_W>>  INIT_S1(wr_trans_fin);  // Depack to pack
  
  // Placed on WRITE Packetizer
  outs_table_entry     wr_out_table[1<<dnp::ace::ID_W]; //[(1<<TID_W)];       // Holds the OutStanding Transactions | Hint : TID_W=4 => 16 slots x 16bits (could be less)
  
  // Constructor
  SC_HAS_PROCESS(ace_master_if);
    ace_master_if(sc_module_name name_="ace_master_if")
    :
    sc_module (name_),
    rd_trans_fin  (2),
    wr_trans_fin  (2)
  {
    SC_THREAD(rd_req_pack_job);
    sensitive << clk.pos();
    async_reset_signal_is(rst_n, false);
    
    SC_THREAD(rd_resp_depack_job);
    sensitive << clk.pos();
    async_reset_signal_is(rst_n, false);
    
    SC_THREAD(wr_req_pack_job);
    sensitive << clk.pos();
    async_reset_signal_is(rst_n, false);
    
    SC_THREAD(wr_resp_depack_job);
    sensitive << clk.pos();
    async_reset_signal_is(rst_n, false);
  
    SC_THREAD(snoop_module);
    sensitive << clk.pos();
    async_reset_signal_is(rst_n, false);
  }
  
  //-------------------------------//
  //--- ACE SNOOPING Module ---//
  //-------------------------------//
  void snoop_module () {
    //-- Start of Reset ---//
    ac_out.Reset();
    cr_in.Reset();
    cd_in.Reset();
  
    cache_flit_in.Reset();
    cache_flit_out.Reset();
    
    //-- End of Reset ---//
    wait();
    while(1) {
      creq_flit_t flit_snp_rcv = cache_flit_in.Pop();
      
      sc_uint<dnp::D_W> sender = (flit_snp_rcv.data[0] >> dnp::S_PTR) & ((1<<dnp::S_W)-1);
      
      ace5_::AC snoop_req;
      snoop_req.prot  = (flit_snp_rcv.data[2] >> dnp::ace::creq::C_PROT_PTR) & ((1<<dnp::ace::C_PROT_W)-1);
      snoop_req.snoop = (flit_snp_rcv.data[1] >> dnp::ace::creq::SNP_PTR) & ((1<<dnp::ace::SNP_W)-1);
      snoop_req.addr  = ((((flit_snp_rcv.data[2]>>dnp::ace::creq::AH_PTR) & ((1<<dnp::ace::AH_W)-1)) << dnp::ace::AL_W) |
                          ((flit_snp_rcv.data[1]>>dnp::ace::creq::AL_PTR) & ((1<<dnp::ace::AL_W)-1)));
      
      NVHLS_ASSERT_MSG(((flit_snp_rcv.data[0].to_uint() >> dnp::D_PTR) & ((1<<dnp::D_W)-1)) == (THIS_ID.read().to_uint()), "Flit misrouted!");
      ac_out.Push(snoop_req);
      
      ace5_::CR snoop_resp = cr_in.Pop();
      cresp_flit_t resp_flit;
      
      resp_flit.data[0] = ((sc_uint<dnp::PHIT_W>) snoop_resp.resp          << dnp::ace::cresp::C_RESP_PTR) |
                          ((sc_uint<dnp::PHIT_W>)dnp::PACK_TYPE__RD_REQ << dnp::T_PTR       ) |
                          ((sc_uint<dnp::PHIT_W>) 0                     << dnp::Q_PTR       ) |
                          ((sc_uint<dnp::PHIT_W>)sender                  << dnp::D_PTR       ) |
                          ((sc_uint<dnp::PHIT_W>)THIS_ID                << dnp::S_PTR       ) |
                          ((sc_uint<dnp::PHIT_W>)0                      << dnp::V_PTR       ) ;
      
      bool has_data = (snoop_resp.resp & 1);
      if (has_data) {
        resp_flit.type = HEAD;
        cache_flit_out.Push(resp_flit);
        ace5_::CD snoop_data;
        do {
          unsigned char data_bytes[ace5_::C_CACHE_WIDTH/8];
          snoop_data = cd_in.Pop();
          duth_fun<ace5_::CD::Data, ace5_::C_CACHE_WIDTH/8>::assign_ac2char(data_bytes, snoop_data.data);
          for(unsigned i=0; i<(ace5_::C_CACHE_WIDTH/8)/2; ++i) {
            resp_flit.data[i] = ((sc_uint<dnp::PHIT_W>)snoop_data.last      << dnp::ace::wdata::LA_PTR ) | // MSB
                                ((sc_uint<dnp::PHIT_W>)data_bytes[(i<<1)+1] << dnp::ace::wdata::B1_PTR ) |
                                ((sc_uint<dnp::PHIT_W>)data_bytes[(i<<1)  ] << dnp::ace::wdata::B0_PTR ) ;
          }
          resp_flit.type = snoop_data.last ? TAIL : BODY;
          cache_flit_out.Push(resp_flit);
        } while (!snoop_data.last);
      } else {
        resp_flit.type = SINGLE;
        cache_flit_out.Push(resp_flit);
      }
    }
  
  }
  
  //-------------------------------//
  //--- READ REQuest Packetizer ---//
  //-------------------------------//
  void rd_req_pack_job () {
    //-- Start of Reset ---//
    // rd_out_table contains outstanding info for each TID, to decide if reordering is possible.
    for (int i=0; i<1<<dnp::ace::ID_W; ++i) {
      rd_out_table[i].dst_last = 0;
      rd_out_table[i].sent     = 0;
      rd_out_table[i].reorder  = false;
    }
    
    ar_in.Reset();
    rd_flit_out.Reset();
    
    ace5_::AddrPayload this_req;
    //-- End of Reset ---//
    wait();
    while(1) {
      
      // New Request from MASTER received
      if(ar_in.PopNB(this_req)) {
        // A new request must stall until it is eligible to depart.
        // Depending the reordering scheme
        // 0 : all in-flight transactions must be to the same destination
        // 1 : all in-flight transactions of the SAME ID, must be to the same destination
        outs_table_entry sel_entry = rd_out_table[this_req.id.to_uint()];
        bool is_coherent = (this_req.snoop > 0) || ((this_req.snoop ==0) && (this_req.domain.xor_reduce()));
        
        // resolve address to node-id
        sc_uint<dnp::D_W> this_dst = is_coherent ? (cfg::SLAVE_NUM+cfg::ALL_MASTER_NUM) : addr_lut_rd(this_req.addr);
        // Check reorder conditions for received TID.
        bool          may_reorder = (sel_entry.sent>0) && (sel_entry.dst_last != this_dst);
        // In case of possible reordering wait_for has the number of transactions
        //   of the same ID, this request has to wait for.
        sc_uint<LOG_MAX_OUTS> wait_for    =  sel_entry.sent;
        
        // Poll for Finished transactions until no longer reordering is possible.
        while(may_reorder || rd_flit_out.Full()) {
          sc_uint<dnp::ace::ID_W> tid_fin;
          if(rd_trans_fin.nb_read(tid_fin)) {
            rd_out_table[tid_fin].sent--;                   // update outstanding table
            if(tid_fin==this_req.id.to_uint()) wait_for--;  // update local wait value
          }
          may_reorder = (wait_for>0);
          wait();
        }; // End of while reorder
        
        // --- Start Packetization --- //
        // Packetize request into a flit. The fields are described in DNP20
        rreq_flit_t tmp_flit;
        tmp_flit.type = SINGLE; // Entire request fits in at single flits thus SINGLE
        tmp_flit.data[0] = ((sc_uint<dnp::PHIT_W>)this_req.snoop            << dnp::ace::req::SNP_PTR) |
                           ((sc_uint<dnp::PHIT_W>)this_req.domain           << dnp::ace::req::DOM_PTR) |
                           ((sc_uint<dnp::PHIT_W>)this_req.id               << dnp::ace::req::ID_PTR ) |
                           ((sc_uint<dnp::PHIT_W>)dnp::PACK_TYPE__RD_REQ << dnp::T_PTR      ) |
                           ((sc_uint<dnp::PHIT_W>) 0                     << dnp::Q_PTR      ) |
                           ((sc_uint<dnp::PHIT_W>)this_dst                   << dnp::D_PTR      ) |
                           ((sc_uint<dnp::PHIT_W>)THIS_ID                    << dnp::S_PTR      ) |
                           ((sc_uint<dnp::PHIT_W>)0                      << dnp::V_PTR      ) ;
        
        tmp_flit.data[1] = ((sc_uint<dnp::PHIT_W>)this_req.len                << dnp::ace::req::LE_PTR) |
                           ((sc_uint<dnp::PHIT_W>)(this_req.addr & 0xffff) << dnp::ace::req::AL_PTR) ;
        
        tmp_flit.data[2] = ((sc_uint<dnp::PHIT_W>)this_req.barrier                  << dnp::ace::req::BAR_PTR) |
                           ((sc_uint<dnp::PHIT_W>)this_req.burst                    << dnp::ace::req::BU_PTR ) |
                           ((sc_uint<dnp::PHIT_W>)this_req.size                     << dnp::ace::req::SZ_PTR ) |
                           ((sc_uint<dnp::PHIT_W>)(this_req.addr >> dnp::ace::AL_W) << dnp::ace::req::AH_PTR ) ;
        
        // Update the table due to the new outstanding
        rd_out_table[this_req.id.to_uint()].sent++;
        rd_out_table[this_req.id.to_uint()].dst_last  = this_dst;
        // send header flit to NoC
        rd_flit_out.Push(tmp_flit); // We've already checked that !Full thus this should not block.
        
      } else { // No RD Req from Master, simply check for finished Outstanding trans
        sc_uint<dnp::ace::ID_W> tid_fin;
        if(rd_trans_fin.nb_read(tid_fin)) {
          rd_out_table[tid_fin].sent--; // update outstanding table
        }
        wait();
      }
    } // End of while(1)
  }; // End of Read Request Packetizer
  
  //-----------------------------------//
  //--- READ RESPonce DE-Packetizer ---//
  //-----------------------------------//
  void rd_resp_depack_job () {
    rack_in.Reset();
    r_out.Reset();
    rd_flit_in.Reset();
    rack_flit_out.Reset();
    while(1) {
      // Blocking read a flit to start depacketize the response.
      rresp_flit_t flit_rcv;
      flit_rcv = rd_flit_in.Pop();
      
      // Construct the transaction's attributes to build the response accordingly.
      ace5_::AddrPayload   active_trans;
      active_trans.id    = (flit_rcv.data[0] >> dnp::ace::rresp::ID_PTR) & ((1 << dnp::ace::ID_W) - 1);
      active_trans.burst = (flit_rcv.data[0] >> dnp::ace::rresp::BU_PTR) & ((1 << dnp::ace::BU_W) - 1);
      active_trans.size  = (flit_rcv.data[1] >> dnp::ace::rresp::SZ_PTR) & ((1 << dnp::ace::SZ_W) - 1);
      active_trans.len   = (flit_rcv.data[1] >> dnp::ace::rresp::LE_PTR) & ((1 << dnp::ace::LE_W) - 1);
      
      unsigned sender      = flit_rcv.get_src();
      bool     is_coherent = (flit_rcv.get_type() == dnp::PACK_TYPE__C_RD_RESP);;
      
      sc_uint<dnp::ace::SZ_W> final_size        = (unsigned) active_trans.size; // Just the size. more compact
      // Partial lower 8-bit part of address to calculate the initial axi pointer in case of a non-aligned address
      sc_uint<dnp::ace::AP_W> addr_part         = (flit_rcv.data[1] >> dnp::ace::rresp::AP_PTR) & ((1<<dnp::ace::AP_W) - 1);
      sc_uint<dnp::ace::AP_W> addr_init_aligned = ((addr_part & (cfg::RD_LANES-1)) & ~((1<<final_size)-1));
      
      // Data Depacketization happens in a loop. Each iteration pops a flit and constructs a beat.
      //   Each iteration transfers data bytes from the flit to the AXI beat.
      //   bytes_per_iter bytes may be transfered, which is limited by two factors
      //   depending the AXI beat size and the bytes in the flit.
      //    1) The available data bytes in the flit is less than the required for the beat
      //    2) The remaining byte lanes are less than the available in the flit
      // For case (1) the flit is emptied and the next flit is popped at the next iteration
      // For case (2) the beat is pushed to Master and the next beat starts in the next iteration
      
      
      // For data Depacketization loop, we keep 2 pointers.
      //   axi_lane_ptr  -> to keep track axi byte lanes to place to data
      //   flit_phit_ptr -> to point at the data of the flit
      sc_uint<8>        axi_lane_ptr   = addr_init_aligned;  // Bytes MOD axi size
      cnt_phit_rresp_t  flit_phit_ptr  = 0;                  // Bytes MOD phits in flit
      // Also we keep track the processed and total data.
      sc_uint<16>  bytes_total    = ((active_trans.len.to_uint()+1)<<final_size);
      sc_uint<16>  bytes_depacked = 0;                                  // Number of DE-packetized bytes
      
      unsigned char resp_build_tmp[cfg::RD_LANES];
      #pragma hls_unroll yes
      for(int i=0; i<cfg::RD_LANES; ++i) resp_build_tmp[i] = 0;
      #pragma hls_pipeline_init_interval 1
      #pragma pipeline_stall_mode flush
      gather_wr_beats : while (1) {
        // Each iteration moves data from the flit the the appropriate place on the AXI RD response
        // The two flit and axi pointers orchistrate the operation, until completion
        sc_uint<8> bytes_axi_left  = ((1<<final_size) - (axi_lane_ptr & ((1<<final_size)-1)));
        sc_uint<8> bytes_flit_left = ((cfg::RRESP_PHITS<<1) - (flit_phit_ptr<<1));
        sc_uint<8> bytes_per_iter  = (bytes_axi_left<bytes_flit_left) ? bytes_axi_left : bytes_flit_left;
        
        if(flit_phit_ptr==0)
          flit_rcv = rd_flit_in.Pop();
        
        // Convert flits to axi transfers. this should be synthesize a mux that routes bytes from flit to axi lanes
        #pragma hls_unroll yes
        build_resp: for (int i = 0; i < (cfg::RD_LANES >> 1); ++i) { // i counts AXI Byte Lanes IN PHITS (i.e. Lanes/bytes_in_phit)
        if (i>=(axi_lane_ptr>>1) && i<((axi_lane_ptr+bytes_per_iter)>>1)) {
          cnt_phit_rresp_t loc_flit_ptr = flit_phit_ptr + (i-(axi_lane_ptr>>1));
          resp_build_tmp[(i << 1) + 1] = (flit_rcv.data[loc_flit_ptr] >> dnp::ace::rdata::B1_PTR) & ((1 << dnp::ace::B_W) - 1); // MSB
          resp_build_tmp[(i << 1)    ] = (flit_rcv.data[loc_flit_ptr] >> dnp::ace::rdata::B0_PTR) & ((1 << dnp::ace::B_W) - 1); // LSB
        }
      }
        
        // transaction event flags
        bool done_job  = ((bytes_depacked+bytes_per_iter)==bytes_total);             // All bytes are processed
        bool done_flit = (flit_phit_ptr+(bytes_per_iter>>1)==cfg::RRESP_PHITS);      // Flit got empty
        bool done_axi  = (((bytes_depacked+bytes_per_iter)&((1<<final_size)-1))==0); // Beat got full
        
        // Push the response to MASTER, when either this Beat got the needed bytes or all bytes are transferred
        if( done_job || done_axi ) {
          ace5_::ReadPayload builder_resp;
          builder_resp.id   = active_trans.id;
          builder_resp.resp = (flit_rcv.data[flit_phit_ptr] >> dnp::ace::rdata::RE_PTR) & ((1 << dnp::ace::R_RE_W) - 1);
          builder_resp.last = ((bytes_depacked+bytes_per_iter)==bytes_total);
          duth_fun<ace5_::Data, cfg::RD_LANES>::assign_char2ac(builder_resp.data, resp_build_tmp);
          r_out.Push(builder_resp);
          #pragma hls_unroll yes
          for(int i=0; i<cfg::RD_LANES; ++i) resp_build_tmp[i] = 0;
        }
        
        // Check to either finish transaction or update the pointers for the next iteration
        if (done_job) { // End of transaction
          // Inform Packetizer about finished transaction, and Exit
          rd_trans_fin.write(active_trans.id.to_uint());
          break;
        } else { // Check for finished transactions
          bytes_depacked +=bytes_per_iter;
          flit_phit_ptr = (done_flit) ? 0 : (flit_phit_ptr +(bytes_per_iter>>1));
          axi_lane_ptr  = (active_trans.burst==enc_::AXBURST::FIXED) ? ((axi_lane_ptr+bytes_per_iter) & ((1<<final_size)-1)) + addr_init_aligned :
                                                                       ((axi_lane_ptr+bytes_per_iter) & (cfg::RD_LANES-1)) ;
        }
      } // End of flit gathering loop
      
      ace5_::RACK tmp_rack;
      tmp_rack = rack_in.Pop();
      if (is_coherent) {
        ack_flit_t ack_flit(SINGLE,THIS_ID.read().to_uint(), sender, 1, 0);
        rack_flit_out.Push(ack_flit);
      }
    
    } // End of while(1)
  }; // End of Read Responce Packetizer
  
  //--------------------------------//
  //--- WRITE REQuest Packetizer ---//
  //--------------------------------//
  void wr_req_pack_job () {
    wr_flit_out.Reset();
    aw_in.Reset();
    w_in.Reset();
    
    for (int i=0; i<1<<dnp::ace::ID_W; ++i) {
      wr_out_table[i].dst_last = 0;
      wr_out_table[i].sent     = 0;
      wr_out_table[i].reorder  = false;
    }
    
    ace5_::AddrPayload this_req;
    wait();
    while(1) {
      if(aw_in.PopNB(this_req)) { // New Request
        // A new request must stall until it is eligible to depart.
        // Depending the reordering scheme
        // 0 : all in-flight transactions must be to the same destination
        // 1 : all in-flight transactions of the SAME ID, must be to the same destination
        outs_table_entry sel_entry = wr_out_table[this_req.id.to_uint()];
        bool pass_thru_home = ((this_req.snoop == 0) && (this_req.domain.xor_reduce())) ||
                               (this_req.snoop == 1);
        
        // resolve address to node-id
        sc_uint<dnp::D_W> this_dst = pass_thru_home ? (cfg::SLAVE_NUM+cfg::ALL_MASTER_NUM) : addr_lut_wr(this_req.addr);
        
        // Check reorder conditions for this TID.
        //   In an ordered NoC reorder may occur when there are outstanding trans towards different destinations
        bool may_reorder   = (sel_entry.sent>0) && (sel_entry.dst_last != this_dst);
        sc_uint<LOG_MAX_OUTS> wait_for =  sel_entry.sent; // Counts outstanding transactions to wait for
        
        // Poll Finished transactions until no longer reorder is possible.
        while(may_reorder  || wr_flit_out.Full()) {
          sc_uint<dnp::ace::ID_W> tid_fin;
          if(wr_trans_fin.nb_read(tid_fin)) {
            wr_out_table[tid_fin].sent--;
            if(tid_fin==this_req.id.to_uint()) wait_for--;
          }
          may_reorder = (wait_for>0);
          wait();
        }; // End of while reorder
        
        // --- Start HEADER Packetization --- //
        // Packetize request according DNP20, and send
        rreq_flit_t tmp_flit;
        wreq_flit_t tmp_mule_flit;
        tmp_mule_flit.type    = HEAD;
        tmp_mule_flit.data[0] = ((sc_uint<dnp::PHIT_W>)this_req.snoop              << dnp::ace::req::SNP_PTR) |
                                ((sc_uint<dnp::PHIT_W>)this_req.domain             << dnp::ace::req::DOM_PTR) |
                                ((sc_uint<dnp::PHIT_W>)this_req.id                 << dnp::ace::req::ID_PTR)  |
                                ((sc_uint<dnp::PHIT_W>)dnp::PACK_TYPE__WR_REQ  << dnp::T_PTR)        |
                                ((sc_uint<dnp::PHIT_W>)0                       << dnp::Q_PTR)        |
                                ((sc_uint<dnp::PHIT_W>)this_dst                    << dnp::D_PTR)        |
                                ((sc_uint<dnp::PHIT_W>)THIS_ID                     << dnp::S_PTR)        |
                                ((sc_uint<dnp::PHIT_W>)0                       << dnp::V_PTR)        ;
        
        tmp_mule_flit.data[1] = ((sc_uint<dnp::PHIT_W>)this_req.len                << dnp::ace::req::LE_PTR) |
                                ((sc_uint<dnp::PHIT_W>)(this_req.addr & 0xffff) << dnp::ace::req::AL_PTR) ;
        
        tmp_mule_flit.data[2] = ((sc_uint<dnp::PHIT_W>)this_req.unique                   << dnp::ace::req::UNQ_PTR) |
                                ((sc_uint<dnp::PHIT_W>)this_req.barrier                  << dnp::ace::req::BAR_PTR) |
                                ((sc_uint<dnp::PHIT_W>)this_req.burst                    << dnp::ace::req::BU_PTR)  |
                                ((sc_uint<dnp::PHIT_W>)this_req.size                     << dnp::ace::req::SZ_PTR)  |
                                ((sc_uint<dnp::PHIT_W>)(this_req.addr >> dnp::ace::AL_W) << dnp::ace::req::AH_PTR)  ;
        
        //tmp_mule_flit.data[3] = ((sc_uint<20>) -1);
        
        wr_out_table[this_req.id.to_uint()].sent++;
        wr_out_table[this_req.id.to_uint()].dst_last = this_dst;
        
        // push header flit to NoC
        //wr_flit_out.Push(tmp_mule_flit); // We've already checked that !Full thus this should not block.
        #pragma hls_pipeline_init_interval 1
        #pragma pipeline_stall_mode flush
        while (!wr_flit_out.PushNB(tmp_mule_flit)) {
          sc_uint<dnp::ace::ID_W> tid_fin;
          if(wr_trans_fin.nb_read(tid_fin)) {
            wr_out_table[tid_fin].sent--; // update outstanding table
          }
          wait();
        }
        
        // --- Start DATA Packetization --- //
        // Data Depacketization happens in a loop. Each iteration pops a flit and constructs a beat.
        //   Multiple iterations may be needed either the consume incoming data or fill a flit, which
        //     which depends on the AXI and flit size.
        //   Each iteration transfers data bytes from the flit to the AXI beat.
        //   The processed bytes per iteration is limited by two factors
        //     depending the AXI beat size and the bytes in the flit.
        //    1) The available data bytes in the flit is less than the required for the beat
        //    2) The remaining byte lanes are less than the available in the flit
        // For case (1) the flit is emptied and the next flit is popped at the next iteration
        // For case (2) the beat is pushed to Master and the next beat starts in the next iteration
        
        // calculate the initial axi pointer in case of a non-aligned address to the bus
        sc_uint<8>  addr_init_aligned  = (this_req.addr.to_uint() & (cfg::WR_LANES-1)) & ~((1<<this_req.size.to_uint())-1);
        
        // For data Depacketization we keep 2 pointers.
        //   - One to keep track axi byte lanes to place to data  (axi_lane_ptr)
        //   - One to point at the data of the flit               (flit_phit_ptr)
        sc_uint<8>       axi_lane_ptr  = addr_init_aligned; // Bytes MOD size
        cnt_phit_wreq_t  flit_phit_ptr = 0;                 // Bytes MOD phits in flit
        
        sc_uint<16>  bytes_total  = ((this_req.len.to_uint()+1)<<this_req.size.to_uint());
        sc_uint<16>  bytes_packed = 0;
        
        unsigned char data_build_tmp[cfg::WR_LANES];
        bool          wstrb_tmp[cfg::WR_LANES];
        sc_uint<1>    last_tmp;
        //#pragma hls_pipeline_init_interval 1
        //#pragma pipeline_stall_mode flush
        gather_wr_beats : while (1) {
        // Calculate the bytes transferred in this iteration, depending the available flit bytes and the remaining to the beat
        sc_uint<8> bytes_axi_left  = ((1<<this_req.size.to_uint()) - (axi_lane_ptr & ((1<<this_req.size.to_uint())-1)));
        sc_uint<8> bytes_flit_left = ((cfg::WREQ_PHITS<<1)         - (flit_phit_ptr<<1));
        sc_uint<8> bytes_per_iter  = (bytes_axi_left<bytes_flit_left) ? bytes_axi_left : bytes_flit_left;
        
        // If current beat has been packed, pop next
        if((bytes_packed & ((1<<this_req.size.to_uint())-1))==0) {
          ace5_::WritePayload this_wr;
          this_wr  = w_in.Pop();
          last_tmp = this_wr.last;
          duth_fun<ace5_::Data , cfg::WR_LANES>::assign_ac2char(data_build_tmp , this_wr.data);
          duth_fun<ace5_::Wstrb, cfg::WR_LANES>::assign_ac2bool(wstrb_tmp      , this_wr.wstrb);
        }
        
        // Convert AXI Beats to flits.
        #pragma hls_unroll yes
        for (int i=0; i<cfg::WREQ_PHITS; ++i){ // i counts phits on the flit
          if(i>=flit_phit_ptr && i<(flit_phit_ptr+(bytes_per_iter>>1))) {
            sc_uint<8> loc_axi_ptr = (axi_lane_ptr + ((i-flit_phit_ptr)<<1));
            tmp_mule_flit.data[i] = ((sc_uint<dnp::PHIT_W>)last_tmp                      << dnp::ace::wdata::LA_PTR ) | // MSB
                                    ((sc_uint<dnp::PHIT_W>)wstrb_tmp[loc_axi_ptr+1]      << dnp::ace::wdata::E1_PTR ) |
                                    ((sc_uint<dnp::PHIT_W>)wstrb_tmp[loc_axi_ptr  ]      << dnp::ace::wdata::E0_PTR ) |
                                    ((sc_uint<dnp::PHIT_W>)data_build_tmp[loc_axi_ptr+1] << dnp::ace::wdata::B1_PTR ) | // (i*2) % 4
                                    ((sc_uint<dnp::PHIT_W>)data_build_tmp[loc_axi_ptr  ] << dnp::ace::wdata::B0_PTR ) ;
          }
        }
        
        // transaction event flags
        bool done_job  = ((bytes_packed+bytes_per_iter)==bytes_total);                            // All bytes are processed
        bool done_flit = (flit_phit_ptr+(bytes_per_iter>>1)==cfg::WREQ_PHITS);                    // Flit got empty
        bool done_axi  = (((bytes_packed+bytes_per_iter)&((1<<(this_req.size.to_uint()))-1))==0); // Beat got full
        
        // Push the flit to NoC when either this Flit got the needed bytes or all bytes are transferred
        if(done_job || done_flit) {
          tmp_mule_flit.type = (bytes_packed+bytes_per_iter==bytes_total) ? TAIL : BODY;
          #pragma hls_pipeline_init_interval 1
          #pragma pipeline_stall_mode flush
          while (!wr_flit_out.PushNB(tmp_mule_flit)) {
            sc_uint<dnp::ace::ID_W> tid_fin;
            if(wr_trans_fin.nb_read(tid_fin)) {
              wr_out_table[tid_fin].sent--; // update outstanding table
            }
            wait();
          }
        }
        
        // Check to either finish transaction or update the pointers for the next iteration
        if (done_job) { // End of transaction
          break;
        } else { // Move to next iteration
          bytes_packed  = bytes_packed+bytes_per_iter;
          flit_phit_ptr = (done_flit) ? 0 : (flit_phit_ptr +(bytes_per_iter>>1));
          axi_lane_ptr  = ((unsigned)this_req.burst==enc_::AXBURST::FIXED) ? ((axi_lane_ptr+bytes_per_iter) & ((1<<this_req.size.to_uint())-1)) + addr_init_aligned :
                          ((axi_lane_ptr+bytes_per_iter) & (cfg::WR_LANES-1)) ;
        }
      } // End of gather_beats. End of transaction loop
      } else { // When no request, Check for finished transactions
        sc_uint<dnp::ace::ID_W> tid_fin;
        if(wr_trans_fin.nb_read(tid_fin)) {
          wr_out_table[tid_fin].sent--;
        }
        wait();
      }
    } // End of While(1)
  }; // End of Read Request Packetizer
  
  
  //------------------------------------//
  //--- WRITE RESPonce DE-Packetizer ---//
  //------------------------------------//
  void wr_resp_depack_job(){
    wr_flit_in.Reset();
    wack_flit_out.Reset();
    b_out.Reset();
    wack_in.Reset();
    wait();
    while(1) {
      wresp_flit_t flit_rcv;
      flit_rcv = wr_flit_in.Pop();
      unsigned sender  = flit_rcv.get_src();
      bool is_coherent = (flit_rcv.get_type() == dnp::PACK_TYPE__C_WR_RESP);
      // Construct the trans Header to create the response
      ace5_::WRespPayload this_resp;
      sc_uint<dnp::ace::ID_W> this_tid = (flit_rcv.data[0] >> dnp::ace::wresp::ID_PTR) & ((1 << dnp::ace::ID_W) - 1);
      this_resp.id = this_tid.to_uint();
      this_resp.resp = (flit_rcv.data[0] >> dnp::ace::wresp::RESP_PTR) & ((1 << dnp::ace::W_RE_W) - 1);
      
      b_out.Push(this_resp);        // Send the response to MASTER
      wr_trans_fin.write(this_tid); // Inform Packetizer for finished transaction
      
      ace5_::WACK tmp_wack;
      tmp_wack = wack_in.Pop();
      if (is_coherent) {
        ack_flit_t ack_flit(SINGLE,THIS_ID.read().to_uint(), sender, 0, 1);
        wack_flit_out.Push(ack_flit);
      }
      
    } // End of While(1)
  }; // End of Write Resp De-pack
  
  
  // Memory map resolving 
  inline unsigned char addr_lut_rd(const ace5_::Addr addr) {
    for (int i=0; i<2; ++i) {
      if (addr>=addr_map[i][0].read() && addr <= addr_map[i][1].read()) return i;
    }
    return 0; // Or send 404
  };
  
  inline unsigned char addr_lut_wr(const ace5_::Addr addr) {
    for (int i=0; i<2; ++i) {
      if (addr>=addr_map[i][0].read() && addr <= addr_map[i][1].read()) return i;
    }
    return 0; // Or send 404
  };
  
}; // End of Master-IF module

#endif // _ACE_MASTER_IF_H_
