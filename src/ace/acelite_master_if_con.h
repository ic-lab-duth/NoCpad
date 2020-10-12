// --------------------------------------------------------- //
//       MASTER-IF Is where the MASTER CONNECTS!!!!!         //
//                                                           //
// Aka. Master <-> Master-IF <-> NoC <-> Slave-IF <-> Slave  //
// --------------------------------------------------------- //


#ifndef _ACELITE_MASTER_IF_H_
#define _ACELITE_MASTER_IF_H_

#include "systemc.h"
#include "nvhls_connections.h"

#include "./ace_master_if_con.h"
#include "../include/ace.h"
#include "../include/axi4_configs_extra.h"
#include "../include/flit_ace.h"
#include "../include/duth_fun.h"

#define LOG_MAX_OUTS 8

#define INIT_S1(n)   n{#n}

// --- Helping Data structures --- //
template <typename cfg>
SC_MODULE(acelite_master_if_con) {
  typedef typename ace::ace5<axi::cfg::ace> ace5_;
  typedef typename axi::AXI4_Encoding       enc_;
  
  typedef flit_dnp<cfg::RREQ_PHITS>  rreq_flit_t;
  typedef flit_dnp<cfg::RRESP_PHITS> rresp_flit_t;
  typedef flit_dnp<cfg::WREQ_PHITS>  wreq_flit_t;
  typedef flit_dnp<cfg::WRESP_PHITS> wresp_flit_t;
  
  typedef sc_uint< nvhls::log2_ceil<cfg::RRESP_PHITS>::val > cnt_phit_rresp_t;
  typedef sc_uint< nvhls::log2_ceil<cfg::WREQ_PHITS>::val >  cnt_phit_wreq_t;
  
  const unsigned char LOG_RD_M_LANES = nvhls::log2_ceil<cfg::RD_LANES>::val;
  const unsigned char LOG_WR_M_LANES = nvhls::log2_ceil<cfg::WR_LANES>::val;
  
  sc_in_clk    clk;
  sc_in <bool> rst_n;
  
  sc_in < sc_uint<(dnp::ace::AH_W+dnp::ace::AL_W)> > addr_map[cfg::SLAVE_NUM][2];
  
  sc_in< sc_uint<dnp::S_W> > THIS_ID;
  
  // AXI MASTER Side Channels
  // --- ACE-LITE --- //
  // NO Snoop Channels
  // --- READ --- //
  Connections::In<ace5_::AddrPayload>    INIT_S1(ar_in);
  Connections::Out<ace5_::ReadPayload>   INIT_S1(r_out);
  // --- WRITE --- //
  Connections::In<ace5_::AddrPayload>    INIT_S1(aw_in);
  Connections::In<ace5_::WritePayload>   INIT_S1(w_in);
  Connections::Out<ace5_::WRespPayload>  INIT_S1(b_out);
  
  // NoC Side Channels
  Connections::Out<rreq_flit_t> INIT_S1(rd_flit_out);
  Connections::In<rresp_flit_t> INIT_S1(rd_flit_in);
  
  Connections::Out<wreq_flit_t> INIT_S1(wr_flit_out);
  Connections::In<wresp_flit_t> INIT_S1(wr_flit_in);
  
  // --- READ Internals --- //
  // FIFOs that pass initiation and finish transactions between Pack-Depack
  sc_fifo<order_info>              INIT_S1(rd_trans_init);
  sc_fifo<sc_uint<dnp::ace::ID_W>> INIT_S1(rd_trans_fin);
  
  // Placed on READ Packetizer
  outs_table_entry    rd_out_table[1<<dnp::ace::ID_W];
  
  // --- WRITE Internals --- //
  sc_fifo<sc_uint<dnp::ace::ID_W>>  INIT_S1(wr_trans_fin);
  
  // Placed on WRITE Packetizer
  outs_table_entry     wr_out_table[1<<dnp::ace::ID_W];
  
  // Constructor
  SC_HAS_PROCESS(acelite_master_if_con);
    acelite_master_if_con(sc_module_name name_="acelite_master_if_con")
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
  }
  
  //-------------------------------//
  //--- NO - ACE SNOOPING Module ---//
  //-------------------------------//
  
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
        // Get info about the outstanding transactions the received request's TID
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
        rd_flit_out.Push(tmp_flit);
        
      } else {
        // No RD Req from Master, simply check for finished Outstanding trans
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
    r_out.Reset();
    rd_flit_in.Reset();
    while(1) {
      rresp_flit_t flit_rcv;
      flit_rcv = rd_flit_in.Pop();
      
      // Construct the transaction's attributes to build the response accordingly.
      ace5_::AddrPayload   active_trans;
      active_trans.id    = (flit_rcv.data[0] >> dnp::ace::rresp::ID_PTR) & ((1 << dnp::ace::ID_W) - 1);
      active_trans.burst = (flit_rcv.data[0] >> dnp::ace::rresp::BU_PTR) & ((1 << dnp::ace::BU_W) - 1);
      active_trans.size  = (flit_rcv.data[1] >> dnp::ace::rresp::SZ_PTR) & ((1 << dnp::ace::SZ_W) - 1);
      active_trans.len   = (flit_rcv.data[1] >> dnp::ace::rresp::LE_PTR) & ((1 << dnp::ace::LE_W) - 1);
      
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
        // Calculate the bytes to transfer in this iteration,
        //   depending the available flit bytes and the remaining to fill the beat
        sc_uint<8> bytes_axi_left  = ((1<<final_size) - (axi_lane_ptr & ((1<<final_size)-1)));
        sc_uint<8> bytes_flit_left = ((cfg::RRESP_PHITS<<1) - (flit_phit_ptr<<1));
        sc_uint<8> bytes_per_iter  = (bytes_axi_left<bytes_flit_left) ? bytes_axi_left : bytes_flit_left;
        
        if(flit_phit_ptr==0)
          flit_rcv = rd_flit_in.Pop();
        
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
        // Get the outstanding info for the received request TID
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
        
        wr_out_table[this_req.id.to_uint()].sent++;
        wr_out_table[this_req.id.to_uint()].dst_last = this_dst;
        
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
        
        // Convert AXI Beats to flits. this should be synthesize a mux that routes bytes from axi lanes to flit
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
    b_out.Reset();
    wait();
    while(1) {
      wresp_flit_t flit_rcv;
      flit_rcv = wr_flit_in.Pop();
      // Construct the trans Header to create the response
      ace5_::WRespPayload this_resp;
      sc_uint<dnp::ace::ID_W> this_tid = (flit_rcv.data[0] >> dnp::ace::wresp::ID_PTR) & ((1 << dnp::ace::ID_W) - 1);
      this_resp.id = this_tid.to_uint();
      this_resp.resp = (flit_rcv.data[0] >> dnp::ace::wresp::RESP_PTR) & ((1 << dnp::ace::W_RE_W) - 1);
      
      b_out.Push(this_resp);        // Send the response to MASTER
      wr_trans_fin.write(this_tid); // Inform Packetizer for finished transaction
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

#endif // _ACELITE_MASTER_IF_H_
