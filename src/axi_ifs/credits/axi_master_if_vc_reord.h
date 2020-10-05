// --------------------------------------------------------- //
//       MASTER-IF Is where the MASTER CONNECTS!!!!!         //
//                                                           //
// Aka. Master <-> Master-IF <-> NoC <-> Slave-IF <-> Slave  //
// --------------------------------------------------------- //


#ifndef AXI4_MASTER_IF_CON_H
#define AXI4_MASTER_IF_CON_H

#include "systemc.h"
#include "nvhls_connections.h"

#include "../../include/flit_axi.h"
#include <axi/axi4.h>

#include "../../include/axi4_configs_extra.h"
#include "../../include/duth_fun.h"

#define LOG_MAX_OUTS 8

// --- Helping Data structures --- //
struct outs_table_entry {
  sc_uint<dnp::D_W>     dst_last;
  sc_uint<LOG_MAX_OUTS> sent;
  bool                  reorder;
};

// Info passed between packetizer and depacketizer to inform about new and finished transactions.
struct order_info {
  sc_uint<dnp::ID_W> tid;
  sc_uint<dnp::D_W>  dst;
  bool          reord;
  unsigned char ticket;
  
  inline friend std::ostream& operator << ( std::ostream& os, const order_info& info ) {
    os <<"TID: "<< info.tid <<", Dst: "<< info.dst <<", Ticket: "<< info.ticket;
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
    sc_trace(tf, info.ticket, name + ".ticket");
  }
#endif
};

// Entries of data storage placed to form a linked list and keep order among TIDs.
template <class FLIT_T>
struct reorder_buff_entry {
  FLIT_T        flit;
  unsigned char nxt_flit;
  bool          valid;
};

// Each TID has a head/tail pointer at the storage buffer, to service each TID independently.
struct reorder_book_entry {
  unsigned char head_flit;
  unsigned char tail_flit;
  unsigned char hol_expect;
};


#define WR_REORD_SLOTS 3
#define RD_REORD_SLOTS 3

// --- Master IF --- //
// AXI Master connects the independent AXI RD and WR cahnnels to the interface 
// The interface gets the Requests and independently packetize and send them into the network
// The Responses are getting depacketized into a seperate thread and are fed back to the MASTER
// Thus Master interface comprises of 4 distinct/parallel blocks WR/RD pack and WR/RD depack
template <typename cfg>
SC_MODULE(axi_master_if_vc) {
  typedef typename axi::axi4<axi::cfg::standard_duth> axi4_;
  typedef typename axi::AXI4_Encoding            enc_;
  
  typedef sc_uint< nvhls::log2_ceil<cfg::VCS>::val > cr_t;
  
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
  
  sc_in < sc_uint<(dnp::AH_W+dnp::AL_W)> > addr_map[cfg::SLAVE_NUM][2];
  
  sc_in< sc_uint<dnp::S_W> > THIS_ID;
  
  // AXI MASTER Side Channels
  // --- READ --- //
  Connections::In<axi4_::AddrPayload>    ar_in{"ar_in"};
  Connections::Out<axi4_::ReadPayload>   r_out{"r_out"};
  
  // --- WRITE --- //
  Connections::In<axi4_::AddrPayload>    aw_in{"aw_in"};
  Connections::In<axi4_::WritePayload>   w_in{"w_in"};
  Connections::Out<axi4_::WRespPayload>  b_out{"b_out"};
  
  // NoC Side Channels
  Connections::Out<rreq_flit_t> rd_flit_data_out{"rd_flit_data_out"};
  Connections::In<cr_t>          rd_flit_cr_in{"rd_flit_cr_in"};
  Connections::In<rresp_flit_t> rd_flit_data_in{"rd_flit_data_in"};
  Connections::Out<cr_t>         rd_flit_cr_out{"rd_flit_cr_out"};
  
  Connections::Out<wreq_flit_t> wr_flit_data_out{"wr_flit_data_out"};
  Connections::In<cr_t>          wr_flit_cr_in{"wr_flit_cr_in"};
  Connections::In<wresp_flit_t> wr_flit_data_in{"wr_flit_data_in"};
  Connections::Out<cr_t>         wr_flit_cr_out{"wr_flit_cr_out"};
  
  
  // --- READ Reordering --- //
  // FIFOs that pass initiation and finish transactions between Pack-Depack
  sc_fifo<order_info> rd_trans_init{"rd_trans_init"};
  sc_fifo<order_info> rd_trans_fin{"rd_trans_fin"};
  
  // Placed on READ Packetizer
  outs_table_entry    rd_out_table[(1<<dnp::ID_W)];
  bool                rd_reord_avail[RD_REORD_SLOTS];
  // Placed on READ De-Packetizer
  reorder_buff_entry<rresp_flit_t>  rd_reord_buff[RD_REORD_SLOTS]; // The Reorder buffer storage, plus link list metadata
  reorder_book_entry                rd_reord_book[(1<<dnp::ID_W)];     // Bookeeping information of the linked list
  
  // --- WRITE Reordering --- //
  sc_fifo<order_info>  wr_trans_init{"wr_trans_init"};
  sc_fifo<order_info>  wr_trans_fin{"wr_trans_fin"};
  
  // Placed on WRITE Packetizer
  outs_table_entry     wr_out_table[(1<<dnp::ID_W)];   // Holds the OutStanding Transactions | Hint : TID_W=4 => 16 slots x 16bits (could be less)
  bool                 wr_reord_avail[WR_REORD_SLOTS]; // Available slots of Reorder Buffer
  // Placed on WRITE De-Packetizer
  reorder_buff_entry<wresp_flit_t> wr_reord_buff[WR_REORD_SLOTS];  // The Reorder buffer storage, plus link list metadata
  reorder_book_entry               wr_reord_book[(1<<dnp::ID_W)];  // Bookeeping information of the linked list
  
  // Constructor
  SC_HAS_PROCESS(axi_master_if_vc);
  axi_master_if_vc(sc_module_name name_="axi_master_if_vc")
    :
    sc_module (name_),
    rd_trans_init (3),
    rd_trans_fin  (3),
    wr_trans_init (3),
    wr_trans_fin  (3) 
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
  //--- READ REQuest Packetizer ---//
  //-------------------------------//
  void rd_req_pack_job () {
    //-- Start of Reset ---//
    for (int i=0; i<(1<<dnp::ID_W); ++i) {
      rd_out_table[i].dst_last = 0;
      rd_out_table[i].sent     = 0;
      rd_out_table[i].reorder  = false;
    }
    
    for (int i=0; i<RD_REORD_SLOTS; ++i) rd_reord_avail[i] = true;
    unsigned char rd_avail_reord_slots = RD_REORD_SLOTS;
    
    sc_uint<cfg::VCS> this_vc;
    unsigned char this_dst    = -1;
    unsigned char this_ticket = -1;
    unsigned char head_ticket = -1;
  
    sc_uint<3> credits_avail[cfg::VCS];
    #pragma hls_unroll yes
    for (int i=0; i<cfg::VCS; ++i) {
      credits_avail[i] = 3;
    }
    
    ar_in.Reset();
    rd_flit_data_out.Reset();
    rd_flit_cr_in.Reset();
    unsigned char total_rd_flits_sent = 0;
    for (int i=0; i<(1<<dnp::ID_W); ++i) rd_out_table[i].dst_last=0;
    
    axi4_::AddrPayload this_req;
    //-- End of Reset ---//
    while(1) {
      wait();
      if(ar_in.PopNB(this_req)) {
        // A new request must stall until it is eligible to depart.
        // Reordering of responses of the same IDs are allowed and handled by the depacketizer in the reorder buffer
        // The response that might get reordered must be able to fit in the buffer
        outs_table_entry sel_entry = rd_out_table[this_req.id.to_uint()]; // Get info for this TID outstandings
        
        this_dst = addr_lut_rd(this_req.addr);
        // Check if reorder may occur
        bool may_reorder   = (sel_entry.sent>0) && (sel_entry.dst_last != this_dst);
        bool through_reord = may_reorder || sel_entry.reorder;
        
        unsigned char wait_for =  sel_entry.sent;
        
        // Calculate the size of the transaction to check if its able to fit in the reorder buffer
        unsigned int bytes_total = ((this_req.len+1)<<this_req.size.to_uint());
        unsigned int phits_total = (bytes_total>>1) + 4; // each phit stores 2 bytes PLUS 2 header phits.
        unsigned int flits_total = (phits_total & 0x3) ? (phits_total>>2)+1 : (phits_total>>2);
  
        this_vc = 0;
        
        // All needed slots must available beforehand, thus wait until space has been freed or its no longer possible to be reordered
        while((through_reord && (rd_avail_reord_slots < flits_total)) || (credits_avail[this_vc]==0) || (total_rd_flits_sent>9)) {
          order_info rcv_fin;
          if(rd_trans_fin.nb_read(rcv_fin)) {
            if(rd_out_table[rcv_fin.tid].sent==1) rd_out_table[rcv_fin.tid].reorder = false;
            rd_out_table[rcv_fin.tid].sent = rd_out_table[rcv_fin.tid].sent - 1;
            total_rd_flits_sent--;
            
            if (rcv_fin.ticket<RD_REORD_SLOTS) {
              rd_reord_avail[rcv_fin.ticket] = true;
              rd_avail_reord_slots++;
            }
          }
  
          cr_t vc_upd;
          if(rd_flit_cr_in.PopNB(vc_upd)) credits_avail[vc_upd]++;
          
          wait();
  
          sel_entry = rd_out_table[this_req.id.to_uint()];
          may_reorder   = (sel_entry.sent>0) && (sel_entry.dst_last != this_dst);
          through_reord = may_reorder || sel_entry.reorder;
        }; // End of while reorder
  
        
        // Required space is guaranteed. Get tickets for the reorder buffer and
        //   inform the depacketizer to set the reorder buffer
        order_info trans_expect;
        trans_expect.tid    = this_req.id.to_uint();
        trans_expect.dst    = this_dst;
        
        // get the reorder tickets
        head_ticket = -1;
        if (through_reord && RD_REORD_SLOTS) { // loop to get tickets and inform Depack
          for(unsigned int tct=0; tct<flits_total; tct++) {
            rd_out_table[this_req.id.to_uint()].reorder = true;
            rd_avail_reord_slots--;
  
            //#pragma hls_unroll yes
            for (int i = 0; i < RD_REORD_SLOTS; ++i) {
              if (rd_reord_avail[i]) {
                this_ticket = i;
                rd_reord_avail[i] = false;
                break;
              }
            }
            if (head_ticket >= RD_REORD_SLOTS) head_ticket = this_ticket;
            trans_expect.reord  = true;
            trans_expect.ticket = this_ticket;
            
            // Send the allocated slot (ticket) to depacketizer, and update local info
            rd_out_table[this_req.id.to_uint()].dst_last = this_dst;
            rd_out_table[this_req.id.to_uint()].sent = rd_out_table[this_req.id.to_uint()].sent + 1;
            total_rd_flits_sent++;
            rd_trans_init.write(trans_expect);
            
            order_info rcv_fin;
            if (rd_trans_fin.nb_read(rcv_fin)) {
              if (rd_out_table[rcv_fin.tid].sent == 1) rd_out_table[rcv_fin.tid].reorder = false;
              rd_out_table[rcv_fin.tid].sent = rd_out_table[rcv_fin.tid].sent - 1;
              total_rd_flits_sent--;
    
              if (rcv_fin.ticket < RD_REORD_SLOTS) {
                rd_reord_avail[rcv_fin.ticket] = true;
                rd_avail_reord_slots++;
              }
            } // end of fin queue checking
            
            wait();
          }
        } else {
          // No reorder is possible - no tickets are needed
          trans_expect.reord  = false;
          trans_expect.ticket = flits_total;
          
          rd_out_table[this_req.id.to_uint()].dst_last = this_dst;
          rd_out_table[this_req.id.to_uint()].sent     = rd_out_table[this_req.id.to_uint()].sent + flits_total;
          total_rd_flits_sent += flits_total;
          rd_trans_init.write(trans_expect);
        }; // End of while reorder
        
        
      } else {
        // If no initiating RD Req from Master, check for finished Outstanding trans
        order_info rcv_fin;
        if(rd_trans_fin.nb_read(rcv_fin)) {
          if(rd_out_table[rcv_fin.tid].sent==1) rd_out_table[rcv_fin.tid].reorder = false;
          rd_out_table[rcv_fin.tid].sent = rd_out_table[rcv_fin.tid].sent - 1;
          total_rd_flits_sent--;
          
          if (rcv_fin.ticket<RD_REORD_SLOTS) {
            rd_reord_avail[rcv_fin.ticket] = true;
            rd_avail_reord_slots++;
          }
        }
  
        cr_t vc_upd;
        if(rd_flit_cr_in.PopNB(vc_upd)) credits_avail[vc_upd]++;
        
        continue;
      }
      
      // The required conditions are met so...
      
      // --- Start Packetization --- //
      // Packetize request into a flit. The fields are described in DNP20
      rreq_flit_t tmp_flit;
      tmp_flit.type = SINGLE; // all request fits in at single flits thus SINGLE
      tmp_flit.vc   = this_vc;
      tmp_flit.data[0] = ((sc_uint<dnp::PHIT_W>)head_ticket            << dnp::req::REORD_PTR)|
                         ((sc_uint<dnp::PHIT_W>)this_req.id            << dnp::req::ID_PTR)   |
                         ((sc_uint<dnp::PHIT_W>)dnp::PACK_TYPE__RD_REQ << dnp::T_PTR)         |
                         ((sc_uint<dnp::PHIT_W>)0                      << dnp::Q_PTR)         |
                         ((sc_uint<dnp::PHIT_W>)this_dst               << dnp::D_PTR)         |
                         ((sc_uint<dnp::PHIT_W>)THIS_ID.read()             << dnp::S_PTR)         ;
      
      tmp_flit.data[1] = ((sc_uint<dnp::PHIT_W>)this_req.len                << dnp::req::LE_PTR) |
                         ((sc_uint<dnp::PHIT_W>) this_req.addr & 0xffff) << dnp::req::AL_PTR;
  
      tmp_flit.data[2] = ((sc_uint<dnp::PHIT_W>)this_req.burst   << dnp::req::BU_PTR)        |
                         ((sc_uint<dnp::PHIT_W>)this_req.size    << dnp::req::SZ_PTR)        |
                         ((sc_uint<dnp::PHIT_W>)(this_req.addr >> dnp::AL_W) << dnp::req::AH_PTR ) ;
      
      // Try to push to Network, but continue reading for incoming finished transactions
      while(credits_avail[this_vc]==0) {
        order_info rcv_fin;
        if(rd_trans_fin.nb_read(rcv_fin)) {
          if(rd_out_table[rcv_fin.tid].sent==1) rd_out_table[rcv_fin.tid].reorder = false;
          rd_out_table[rcv_fin.tid].sent = rd_out_table[rcv_fin.tid].sent - 1;
          total_rd_flits_sent--;
          
          if (rcv_fin.ticket<RD_REORD_SLOTS) {
            rd_reord_avail[rcv_fin.ticket] = true;
            rd_avail_reord_slots++;
          }
        }
  
        cr_t vc_upd;
        if (rd_flit_cr_in.PopNB(vc_upd)) credits_avail[vc_upd]++;
        wait();
      }
      bool dbg_rreq_ok = rd_flit_data_out.PushNB(tmp_flit); // We've already checked that !Full thus this should not block.
      NVHLS_ASSERT_MSG(dbg_rreq_ok, "R Req pack DROP!!!");
      credits_avail[this_vc]--;
    } // End of while(1)
  }; // End of Read Request Packetizer
  
  //-----------------------------------//
  //--- READ RESPonce DE-Packetizer ---//
  //-----------------------------------//
  void rd_resp_depack_job () {
    //--- Start of Reset ---//
    const unsigned int MAX_RD_FLITS  = 1024;
    
    for (int i=0; i<(1<<dnp::ID_W); ++i){
      rd_reord_book[i].tail_flit  = -1;
      rd_reord_book[i].head_flit  = -1;
      rd_reord_book[i].hol_expect = 0;
    }
    for (int i=0; i<RD_REORD_SLOTS; ++i){
      rd_reord_buff[i].nxt_flit = -1;
      rd_reord_buff[i].valid    = false;
    }
    
    bool          has_active_trans = false;
    axi4_::AddrPayload   active_trans;
    unsigned char active_trans_dst = -1;
    
    bool         bypass_valid  = false;
    bool         bypass_active = false;
    rresp_flit_t bypass_flit;
    
    bool   reord_active = false;
    unsigned char reord_slot_active = -1;
    unsigned char rcv_ticket_nxt    = -1;
    
    unsigned int phits_data_count = 0;
  
    rresp_flit_t cur_flit;
    
    r_out.Reset();
    rd_flit_data_in.Reset();
    rd_flit_cr_out.Reset();
    sc_uint<dnp::SZ_W> final_size;
    sc_uint<dnp::AP_W> addr_part;
    sc_uint<dnp::AP_W> addr_init_aligned;
    sc_uint<8>        axi_lane_ptr;
    cnt_phit_rresp_t  flit_phit_ptr;
    sc_uint<16>  bytes_total;
    sc_uint<16>  bytes_depacked;
    
    unsigned char resp_build_tmp[cfg::RD_LANES];
    axi4_::ReadPayload builder_resp;
    //--- End of Reset ---//
    #pragma hls_pipeline_init_interval 1
    #pragma pipeline_stall_mode flush
    while(1) {
      wait();
      
      // Set the reorder buffer's state to expect the in-flight responses according the Init transactions
      order_info trans_expect;
      if(rd_trans_init.nb_read(trans_expect)) {
        if(trans_expect.reord) {
          // PUSH NEW item to linked list
          if(rd_reord_book[trans_expect.tid].head_flit>=RD_REORD_SLOTS) {
            rd_reord_book[trans_expect.tid].head_flit = trans_expect.ticket;
          }
          
          // Change old tail's nxt_flit to point to new ticket.
          if(rd_reord_book[trans_expect.tid].tail_flit<RD_REORD_SLOTS) {
            rd_reord_buff[rd_reord_book[trans_expect.tid].tail_flit].nxt_flit = trans_expect.ticket;
          }
          // Update the TID's tail.
          rd_reord_book[trans_expect.tid].tail_flit = trans_expect.ticket;
        } else {
          // When no reorder, ticket gives the number of expected flits
          rd_reord_book[trans_expect.tid].hol_expect += trans_expect.ticket; // When no reorder, ticket gives the number of expected flits
        }
      } // End of new trans sink      
      
      // Handle incoming flits. 
      //   Either a packet that bypasses the reorder buffer,
      //   or Store it in the reorder buffer
      if (!bypass_valid) {
        rresp_flit_t flit_rcv;
        if (rd_flit_data_in.PopNB(flit_rcv)) {
          cr_t flit_rcv_vc = flit_rcv.get_vc();
          bool dbg_rresp_cr_ok = rd_flit_cr_out.PushNB(flit_rcv_vc);
          NVHLS_ASSERT_MSG(dbg_rresp_cr_ok, "R Resp credit DROP!!!");
          
          unsigned char rcv_ticket;
          if (flit_rcv.is_head() || flit_rcv.is_single())
            rcv_ticket = ((flit_rcv.data[0] >> (dnp::rresp::REORD_PTR)) & ((1<<dnp::REORD_W)-1));
          else
            rcv_ticket = rcv_ticket_nxt;
          
          // Through reorder when the ticket belongs to its slots
          if(rcv_ticket<RD_REORD_SLOTS && RD_REORD_SLOTS) {
            rd_reord_buff[rcv_ticket].flit  = flit_rcv;
            rd_reord_buff[rcv_ticket].valid = true;
  
            rcv_ticket_nxt = rd_reord_buff[rcv_ticket].nxt_flit;
          } else {
            // Otherwise its a bypass flit
            bypass_flit  = flit_rcv;
            bypass_valid = true;
            rcv_ticket_nxt = -1;
          }
        }
      }
      
      // When the depacketizer does not actively reconstructs a response
      //  Check if there are conditions to initiate either from reorder buffer, or bypass path
      if (!(bypass_active || reord_active)) {
        if(bypass_valid) {
          bypass_active = true;
          cur_flit      = bypass_flit;
        } else if (RD_REORD_SLOTS) {
          // Check ROB for new transactions to initiate
          rresp_flit_t flit_reord;
          #pragma hls_unroll yes
          for (int i=0; i<(1<<dnp::ID_W); ++i) {
            if (rd_reord_book[i].hol_expect==0) {
              if (rd_reord_book[i].head_flit < RD_REORD_SLOTS) {
                if (rd_reord_buff[rd_reord_book[i].head_flit].valid) {
                  // flit_reord
                  cur_flit          = rd_reord_buff[rd_reord_book[i].head_flit].flit;
                  reord_slot_active = rd_reord_book[i].head_flit;
                  reord_active      = true;
                  break;
                }
              }
            }
          } // End of REORDER CHECK
        }
        
        // If either bypass/reorder has valid transaction, get the transaction info
        if (bypass_active || reord_active) {
          active_trans.id    = (cur_flit.data[0] >> dnp::rresp::ID_PTR) & ((1 << dnp::ID_W) - 1);
          active_trans.burst = (cur_flit.data[0] >> dnp::rresp::BU_PTR) & ((1 << dnp::BU_W) - 1);
          active_trans.size  = (cur_flit.data[1] >> dnp::rresp::SZ_PTR) & ((1 << dnp::SZ_W) - 1);
          active_trans.len   = (cur_flit.data[1] >> dnp::rresp::LE_PTR) & ((1 << dnp::LE_W) - 1);
  
          final_size        = (unsigned) active_trans.size; // Just the size. more compact
          // Partial lower 8-bit part of address to calculate the initial axi pointer in case of a non-aligned address
          addr_part         = (cur_flit.data[1] >> dnp::rresp::AP_PTR) & ((1<<dnp::AP_W) - 1);
          addr_init_aligned = ((addr_part & (cfg::RD_LANES-1)) & ~((1<<final_size)-1));
  
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
          axi_lane_ptr   = addr_init_aligned;  // Bytes MOD axi size
          flit_phit_ptr  = 0;                  // Bytes MOD phits in flit
          
          // Also we keep track the processed and total data.
          bytes_total    = ((active_trans.len.to_uint()+1)<<final_size);
          bytes_depacked = 0;                                  // Number of DE-packetized bytes
          
          active_trans_dst = (cur_flit.data[0] >> dnp::S_PTR) & ((1<<dnp::S_W)-1); //cur_flit.get_src();
  
          // Drop the head Flit and update the info
          unsigned char this_ticket;
          if(bypass_active) {
            rd_reord_book[active_trans.id.to_uint()].hol_expect--;
            bypass_valid = false;
            this_ticket  = -1;
          } else if (RD_REORD_SLOTS) {
            this_ticket = rd_reord_book[active_trans.id.to_uint()].head_flit;
            unsigned char this_nxt_flit = rd_reord_buff[rd_reord_book[active_trans.id.to_uint()].head_flit].nxt_flit;
    
            reord_slot_active = this_nxt_flit;
    
            // List Update
            rd_reord_buff[rd_reord_book[active_trans.id.to_uint()].head_flit].valid    = false;
            rd_reord_buff[rd_reord_book[active_trans.id.to_uint()].head_flit].nxt_flit = -1;     // Probably not needed
    
            if (rd_reord_book[active_trans.id.to_uint()].head_flit == rd_reord_book[active_trans.id.to_uint()].tail_flit) {
              rd_reord_book[active_trans.id.to_uint()].tail_flit = -1;
            }
    
            rd_reord_book[active_trans.id.to_uint()].head_flit = this_nxt_flit;
          }
  
          order_info fin_trans;
          fin_trans.tid    = active_trans.id.to_uint();
          fin_trans.ticket = this_ticket;
          fin_trans.dst    = active_trans_dst;
  
          rd_trans_fin.write(fin_trans);
        }
      } else if ((bypass_active && bypass_valid) || (reord_active && rd_reord_buff[reord_slot_active].valid)) {
        // After the initiation of transaction, handle the rest of the flits
        
        if(reord_active) cur_flit = rd_reord_buff[rd_reord_book[active_trans.id.to_uint()].head_flit].flit;
        else             cur_flit = bypass_flit;
  
        // Calculate the bytes to transfer in this iteration
        sc_uint<8> bytes_axi_left  = ((1<<final_size) - (axi_lane_ptr & ((1<<final_size)-1)));
        sc_uint<8> bytes_flit_left = ((cfg::RRESP_PHITS<<1) - (flit_phit_ptr<<1));
        sc_uint<8> bytes_per_iter  = (bytes_axi_left<bytes_flit_left) ? bytes_axi_left : bytes_flit_left;
  
        // Convert flits to AXI Beats.
        #pragma hls_unroll yes
        build_resp: for (int i = 0; i < (cfg::RD_LANES >> 1); ++i) { // i counts AXI Byte Lanes IN PHITS (i.e. Lanes/bytes_in_phit)
          if (i >= (axi_lane_ptr >> 1) && i < ((axi_lane_ptr + bytes_per_iter) >> 1)) {
            cnt_phit_rresp_t loc_flit_ptr = flit_phit_ptr + (i - (axi_lane_ptr >> 1));
            resp_build_tmp[(i << 1) + 1] =
                    (cur_flit.data[loc_flit_ptr] >> dnp::rdata::B1_PTR) & ((1 << dnp::B_W) - 1); // MSB
            resp_build_tmp[(i << 1)] = (cur_flit.data[loc_flit_ptr] >> dnp::rdata::B0_PTR) & ((1 << dnp::B_W) - 1); // LSB
          }
        }
        
        // transaction event flags
        bool done_job  = ((bytes_depacked+bytes_per_iter)==bytes_total);             // All bytes are processed
        bool done_flit = (flit_phit_ptr+(bytes_per_iter>>1)==cfg::RRESP_PHITS);      // Flit got empty
        bool done_axi  = (((bytes_depacked+bytes_per_iter)&((1<<final_size)-1))==0); // Beat got full
  
  
        // Drop the flit when Flit is full or all data have been consumed, and inform packetizer
        if( done_job || done_flit ) {
          unsigned char this_ticket;
          if(bypass_active) {
            rd_reord_book[active_trans.id.to_uint()].hol_expect--;
            bypass_valid = false;
            this_ticket  = -1;
          } else if (RD_REORD_SLOTS) {
            this_ticket = rd_reord_book[active_trans.id.to_uint()].head_flit;
            unsigned char this_nxt_flit = rd_reord_buff[rd_reord_book[active_trans.id.to_uint()].head_flit].nxt_flit;
            
            reord_slot_active = this_nxt_flit;
      
            // List Update
            rd_reord_buff[rd_reord_book[active_trans.id.to_uint()].head_flit].valid    = false;
            rd_reord_buff[rd_reord_book[active_trans.id.to_uint()].head_flit].nxt_flit = -1;     // Probably not needed
      
            if (rd_reord_book[active_trans.id.to_uint()].head_flit == rd_reord_book[active_trans.id.to_uint()].tail_flit) {
              rd_reord_book[active_trans.id.to_uint()].tail_flit = -1;
            }
      
            rd_reord_book[active_trans.id.to_uint()].head_flit = this_nxt_flit;
          }
    
          order_info fin_trans;
          fin_trans.tid    = active_trans.id.to_uint();
          fin_trans.ticket = this_ticket;
          fin_trans.dst    = active_trans_dst;
    
          rd_trans_fin.write(fin_trans);
        }
        
        // Push the response when its gets the appropriate data
        if( done_job || done_axi ) {
          builder_resp.id   = active_trans.id;
          builder_resp.resp = (cur_flit.data[flit_phit_ptr] >> dnp::rdata::RE_PTR) & ((1 << dnp::RE_W) - 1);
          builder_resp.last = ((bytes_depacked+bytes_per_iter)==bytes_total);
          duth_fun<axi4_::Data, cfg::RD_LANES>::assign_char2ac(builder_resp.data, resp_build_tmp);
          r_out.Push(builder_resp);
          #pragma hls_unroll yes
          for(int i=0; i<cfg::RD_LANES; ++i) resp_build_tmp[i] = 0;
        }
        
        if(done_job) { // End of transaction
          bypass_active    = false;
          reord_active     = false;
          bytes_depacked   = 0;
        } else {
          // Response continues, update pointers.
          bytes_depacked += bytes_per_iter;
          flit_phit_ptr  = (done_flit) ? 0 : (flit_phit_ptr +(bytes_per_iter>>1));
          axi_lane_ptr   = (active_trans.burst==enc_::AXBURST::FIXED) ? ((axi_lane_ptr+bytes_per_iter) & ((1<<final_size)-1)) + addr_init_aligned :
                           ((axi_lane_ptr+bytes_per_iter) & (cfg::RD_LANES-1)) ;
        }
      } // End of transaction handling
    } // End of while(1)
  }; // End of Read Responce Packetizer
    
  //--------------------------------//
  //--- WRITE REQuest Packetizer ---//
  //--------------------------------//
  void wr_req_pack_job () {
    wr_flit_data_out.Reset();
    wr_flit_cr_in.Reset();
    aw_in.Reset();
    w_in.Reset();
  
    sc_uint<3> wr_credits_avail[cfg::VCS];
    for (int i=0; i<cfg::VCS; ++i) {
      wr_credits_avail[i] = 3;
    }

    for (int i=0; i<(1<<dnp::ID_W); ++i) {
      wr_out_table[i].dst_last = 0;
      wr_out_table[i].sent     = 0;
      wr_out_table[i].reorder  = false;
    }
    for (int i=0; i<WR_REORD_SLOTS; ++i) wr_reord_avail[i] = true;
    unsigned char wr_avail_reord_slots = WR_REORD_SLOTS;
    unsigned char this_ticket          = -1;
    unsigned char this_dst             = -1;
    sc_uint<cfg::VCS> this_vc;
    
    while(1) {
      wait();
      
      // Always check for finished transactions
      order_info    rcv_fin;
      if(wr_trans_fin.nb_read(rcv_fin)) {
        if(wr_out_table[rcv_fin.tid].sent==1) wr_out_table[rcv_fin.tid].reorder = false;
        wr_out_table[rcv_fin.tid].sent = wr_out_table[rcv_fin.tid].sent - 1;
    
        if (rcv_fin.ticket<WR_REORD_SLOTS) {
          wr_reord_avail[rcv_fin.ticket] = true;
          wr_avail_reord_slots++;
        }
      }
  
      cr_t vc_upd;
      if(wr_flit_cr_in.PopNB(vc_upd)) wr_credits_avail[vc_upd]++;
      
      axi4_::AddrPayload this_req;
      if(aw_in.PopNB(this_req)) {
        // A new request must stall until it is eligible to depart.
        // Reordering of responses of the same IDs are allowed and handled by the depacketizer in the reorder buffer
        // The response that might get reordered must be able to fit in the buffer
        outs_table_entry sel_entry = wr_out_table[this_req.id.to_uint()];
        this_dst = addr_lut_wr(this_req.addr);
    
        bool          may_reorder   = (sel_entry.sent>0) && (sel_entry.dst_last != this_dst);
        bool          through_reord = may_reorder || sel_entry.reorder;
        unsigned char wait_for      =  sel_entry.sent;
  
        this_vc = 0;
        // Stall until the necessary resources are available
        while(through_reord && (wr_avail_reord_slots<1)) {
          order_info rcv_fin;
          if(wr_trans_fin.nb_read(rcv_fin)) {
            if(wr_out_table[rcv_fin.tid].sent==1) wr_out_table[rcv_fin.tid].reorder = false;
            wr_out_table[rcv_fin.tid].sent = wr_out_table[rcv_fin.tid].sent - 1;
            
            if (rcv_fin.ticket<WR_REORD_SLOTS && WR_REORD_SLOTS) {
              wr_reord_avail[rcv_fin.ticket] = true;
              wr_avail_reord_slots++;
            }
          }
          wait();
        }; // End of while reorder
        
        // Get ticket
        this_ticket = -1;
        if (through_reord) {
          wr_out_table[this_req.id.to_uint()].reorder = true;
          wr_avail_reord_slots--;
          
          for (int i=0; i<WR_REORD_SLOTS; ++i) {
            if(wr_reord_avail[i]) {
              this_ticket = i;
              wr_reord_avail[i] = false;
              break;
            }
          }
        }
      } else {
        // No available response
        
        continue;
      }
      
      // --- Start HEADER Packetization --- //
      wreq_flit_t tmp_flit;
      wreq_flit_t tmp_mule_flit;
      tmp_mule_flit.type    = HEAD;
      tmp_mule_flit.vc      = this_vc;
      tmp_mule_flit.data[0] = ((sc_uint<dnp::PHIT_W>)this_ticket            << dnp::req::REORD_PTR) |
                              ((sc_uint<dnp::PHIT_W>)this_req.id            << dnp::req::ID_PTR)    |
                              ((sc_uint<dnp::PHIT_W>)dnp::PACK_TYPE__WR_REQ << dnp::T_PTR)          |
                              ((sc_uint<dnp::PHIT_W>)0                      << dnp::Q_PTR)          |
                              ((sc_uint<dnp::PHIT_W>)this_dst               << dnp::D_PTR)          |
                              ((sc_uint<dnp::PHIT_W>)THIS_ID.read()             << dnp::S_PTR)          ;
  
      tmp_mule_flit.data[1] = ((sc_uint<dnp::PHIT_W>)this_req.len   << dnp::req::LE_PTR) |
                              ((sc_uint<dnp::PHIT_W>)this_req.addr & 0xffff);
      
      tmp_mule_flit.data[2] = ((sc_uint<dnp::PHIT_W>)this_req.burst << dnp::req::BU_PTR)      |
                              ((sc_uint<dnp::PHIT_W>)this_req.size  << dnp::req::SZ_PTR)      |
                              ((sc_uint<dnp::PHIT_W>)(this_req.addr >> dnp::AL_W) << dnp::req::AH_PTR)  ;

      wr_out_table[this_req.id.to_uint()].sent++;
      wr_out_table[this_req.id.to_uint()].dst_last = this_dst;
  
      order_info trans_expect;
      trans_expect.tid    = this_req.id.to_uint();
      trans_expect.dst    = this_dst;
      trans_expect.ticket = this_ticket;
  
      wr_trans_init.write(trans_expect);
      
      // If network is not ready, continue to poll for finished transactions
      #pragma hls_pipeline_init_interval 1
      #pragma pipeline_stall_mode flush
      while(wr_credits_avail[this_vc]==0) {
        order_info rcv_fin;
        if(wr_trans_fin.nb_read(rcv_fin)) {
          if(wr_out_table[rcv_fin.tid].sent==1) wr_out_table[rcv_fin.tid].reorder = false;
          wr_out_table[rcv_fin.tid].sent = wr_out_table[rcv_fin.tid].sent - 1;
    
          if (rcv_fin.ticket<WR_REORD_SLOTS) {
            wr_reord_avail[rcv_fin.ticket] = true;
            wr_avail_reord_slots++;
          }
        }
  
        cr_t vc_upd;
        if(wr_flit_cr_in.PopNB(vc_upd)) wr_credits_avail[vc_upd]++;
        wait();
      };
      bool dbg_wreq_ok = wr_flit_data_out.PushNB(tmp_mule_flit); // We've already checked that !Full thus this should not block.
      NVHLS_ASSERT_MSG(dbg_wreq_ok, "W Req pack DROP!!!");
      wr_credits_avail[this_vc]--;
      wait();
  
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
        wait();
        // Calculate the bytes transferred in this iteration, depending the available flit bytes and the remaining to the beat
        sc_uint<8> bytes_axi_left  = ((1<<this_req.size.to_uint()) - (axi_lane_ptr & ((1<<this_req.size.to_uint())-1)));
        sc_uint<8> bytes_flit_left = ((cfg::WREQ_PHITS<<1)         - (flit_phit_ptr<<1));
        sc_uint<8> bytes_per_iter  = (bytes_axi_left<bytes_flit_left) ? bytes_axi_left : bytes_flit_left;
  
  
        // If current beat has been packed, pop next
        if((bytes_packed & ((1<<this_req.size.to_uint())-1))==0) {
          axi4_::WritePayload this_wr;
          this_wr  = w_in.Pop();
          last_tmp = this_wr.last;
          duth_fun<axi4_::Data , cfg::WR_LANES>::assign_ac2char(data_build_tmp , this_wr.data);
          duth_fun<axi4_::Wstrb, cfg::WR_LANES>::assign_ac2bool(wstrb_tmp      , this_wr.wstrb);
        }
  
        // Convert AXI Beats to flits.
        #pragma hls_unroll yes
        for (int i=0; i<cfg::WREQ_PHITS; ++i){ // i counts phits on the flit
          if(i>=flit_phit_ptr && i<(flit_phit_ptr+(bytes_per_iter>>1))) {
            sc_uint<8> loc_axi_ptr = (axi_lane_ptr + ((i-flit_phit_ptr)<<1));
            tmp_mule_flit.data[i] = ((sc_uint<dnp::PHIT_W>)last_tmp                      << dnp::wdata::LA_PTR ) | // MSB
                                    ((sc_uint<dnp::PHIT_W>)wstrb_tmp[loc_axi_ptr+1]      << dnp::wdata::E1_PTR ) |
                                    ((sc_uint<dnp::PHIT_W>)wstrb_tmp[loc_axi_ptr  ]      << dnp::wdata::E0_PTR ) |
                                    ((sc_uint<dnp::PHIT_W>)data_build_tmp[loc_axi_ptr+1] << dnp::wdata::B1_PTR ) | // (i*2) % 4
                                    ((sc_uint<dnp::PHIT_W>)data_build_tmp[loc_axi_ptr  ] << dnp::wdata::B0_PTR ) ;
          }
        }
        
        // transaction event flags
        bool done_job  = ((bytes_packed+bytes_per_iter)==bytes_total);                            // All bytes are processed
        bool done_flit = (flit_phit_ptr+(bytes_per_iter>>1)==cfg::WREQ_PHITS);                    // Flit got empty
        bool done_axi  = (((bytes_packed+bytes_per_iter)&((1<<(this_req.size.to_uint()))-1))==0); // Beat got full
    
        // If network is not ready, continue to poll for finished transactions
        if(done_job || done_flit) {
          tmp_mule_flit.type = (bytes_packed+bytes_per_iter==bytes_total) ? TAIL : BODY;
          tmp_mule_flit.vc   = this_vc;
          while (wr_credits_avail[this_vc]==0) {
            order_info rcv_fin;
            if(wr_trans_fin.nb_read(rcv_fin)) {
              if(wr_out_table[rcv_fin.tid].sent==1) wr_out_table[rcv_fin.tid].reorder = false;
              wr_out_table[rcv_fin.tid].sent = wr_out_table[rcv_fin.tid].sent - 1;
        
              if (rcv_fin.ticket<WR_REORD_SLOTS) {
                wr_reord_avail[rcv_fin.ticket] = true;
                wr_avail_reord_slots++;
              }
            }
  
            cr_t vc_upd;
            if (wr_flit_cr_in.PopNB(vc_upd)) wr_credits_avail[vc_upd]++;
            wait();
          }
          bool dbg_wreq_ok = wr_flit_data_out.PushNB(tmp_mule_flit); // We've already checked that !Full thus this should not block.
          NVHLS_ASSERT_MSG(dbg_wreq_ok, "W Req pack DROP!!!");
          wr_credits_avail[this_vc]--;
          wait();
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
      }  // End of gather beats
      
    } // End of While(1)
    
  }; // End of Read Request Packetizer
  
  
  //------------------------------------//
  //--- WRITE RESPonce DE-Packetizer ---//
  //------------------------------------//
  void wr_resp_depack_job(){
    wr_flit_data_in.Reset();
    wr_flit_cr_out.Reset();
    b_out.Reset();
    
    for (int i=0; i<(1<<dnp::ID_W); ++i){
      wr_reord_book[i].tail_flit  = -1;
      wr_reord_book[i].head_flit  = -1;
      wr_reord_book[i].hol_expect = 0;
    }
    for (int i=0; i<WR_REORD_SLOTS; ++i){
      wr_reord_buff[i].nxt_flit = -1;
      wr_reord_buff[i].valid    = false;
    }
    
    while(1) {
      wait();
      // Always check for finished transactions
      order_info trans_expect;
      if(wr_trans_init.nb_read(trans_expect)) {
        if(trans_expect.ticket<WR_REORD_SLOTS && WR_REORD_SLOTS) {
          // PUSH NEW item to linked list
          if(wr_reord_book[trans_expect.tid].head_flit>=WR_REORD_SLOTS) {
            wr_reord_book[trans_expect.tid].head_flit = trans_expect.ticket;
          }
          
          // Change old tail's nxt_flit to point to new ticket.
          if(wr_reord_book[trans_expect.tid].tail_flit<WR_REORD_SLOTS) {
            wr_reord_buff[wr_reord_book[trans_expect.tid].tail_flit].nxt_flit = trans_expect.ticket;
          }
          // Update the TID's tail.
          wr_reord_book[trans_expect.tid].tail_flit = trans_expect.ticket;
        } else {
          wr_reord_book[trans_expect.tid].hol_expect++;
        }
      } // End of new trans sink
      
      // Handle incoming flits. 
      //   Either a packet that bypasses the reorder buffer,
      //   or Store it in the reorder buffer
      bool bypass = false;
      wresp_flit_t flit_rcv;
      if(wr_flit_data_in.PopNB(flit_rcv)) {
        cr_t flit_rcv_vc = flit_rcv.get_vc();
        bool dbg_wresp_cr_ok = wr_flit_cr_out.PushNB(flit_rcv_vc);
        NVHLS_ASSERT_MSG(dbg_wresp_cr_ok, "W Resp credit DROP!!!");
        
        unsigned char rcv_ticket = ((flit_rcv.data[0] >> (dnp::wresp::REORD_PTR)) & ((1<<dnp::REORD_W)-1));
        if(rcv_ticket<WR_REORD_SLOTS && WR_REORD_SLOTS) { // Through reorder
          wr_reord_buff[rcv_ticket].flit  = flit_rcv;
          wr_reord_buff[rcv_ticket].valid = true;
        } else {
          bypass = true;
        }
      }
      
      // Send response to Master either from bypass or reorder buffer 
      axi4_::WRespPayload this_resp;
      if(bypass) {
        unsigned char this_tid = (flit_rcv.data[0] >> dnp::wresp::ID_PTR) & ((1<<dnp::ID_W)-1);
        this_resp.id   = this_tid;
        this_resp.resp = (flit_rcv.data[0] >> dnp::wresp::RESP_PTR) & ((1<<dnp::RE_W)-1);
       
        order_info fin_trans;
        fin_trans.tid    = this_tid;
        fin_trans.ticket = (flit_rcv.data[0] >> dnp::wresp::REORD_PTR) & ((1<<dnp::REORD_W)-1);
        fin_trans.dst    = (flit_rcv.data[0] >> dnp::S_PTR) & ((1<<dnp::S_W)-1);
  
        wr_reord_book[this_tid].hol_expect--;
        
        b_out.Push(this_resp);
        wr_trans_fin.write(fin_trans);
      
      } else if (WR_REORD_SLOTS) {
        // Check ROB for valid responses to send to Master
        wresp_flit_t flit_reord;
        bool reord_valid = false;
        #pragma hls_unroll yes
        for (int i=0; i<(1<<dnp::ID_W); ++i) {
          if (wr_reord_book[i].hol_expect==0) {
            if (wr_reord_book[i].head_flit < WR_REORD_SLOTS) {
              if (wr_reord_buff[wr_reord_book[i].head_flit].valid) {
                flit_reord = wr_reord_buff[wr_reord_book[i].head_flit].flit;
                
                unsigned char this_nxt_flit = wr_reord_buff[wr_reord_book[i].head_flit].nxt_flit;
                
                // List Update
                wr_reord_buff[wr_reord_book[i].head_flit].valid    = false;
                wr_reord_buff[wr_reord_book[i].head_flit].nxt_flit = -1;     // Probably not needed
                
                if (wr_reord_book[i].head_flit == wr_reord_book[i].tail_flit) {
                  wr_reord_book[i].tail_flit = -1;
                }
                
                wr_reord_book[i].head_flit = this_nxt_flit;
                // End of - List Update
      
                reord_valid = true;
                break;
              }
            }
          }
        }
        
        if(reord_valid) {
          this_resp.id   = (flit_reord.data[0] >> dnp::wresp::ID_PTR)   & ((1<<dnp::ID_W)-1);
          this_resp.resp = (flit_reord.data[0] >> dnp::wresp::RESP_PTR) & ((1<<dnp::RE_W)-1);
  
          order_info fin_trans;
          fin_trans.tid    = (flit_reord.data[0] >> dnp::wresp::ID_PTR)    & ((1<<dnp::ID_W)-1);
          fin_trans.ticket = (flit_reord.data[0] >> dnp::wresp::REORD_PTR) & ((1<<dnp::REORD_W)-1);
          
          b_out.Push(this_resp);
          wr_trans_fin.write(fin_trans);
        }
      } // End of send flit from ROB
    } // End of While(1)
  }; // End of Write Resp Packetizer  
  
  
  // Memory map resolving
  inline unsigned char addr_lut_rd(const axi4_::Addr addr) {
    for (int i=0; i<cfg::SLAVE_NUM; ++i) {
      if (addr>=addr_map[i][0].read() && addr <= addr_map[i][1].read()) return i;
    }
    NVHLS_ASSERT_MSG(0, "RD address not resolved!");
    return 0; 
  };
  
  inline unsigned char addr_lut_wr(const axi4_::Addr addr) {
    for (int i=0; i<cfg::SLAVE_NUM; ++i) {
      if (addr>=addr_map[i][0].read() && addr <= addr_map[i][1].read()) return i;
    }
    NVHLS_ASSERT_MSG(0, "WR address not resolved!");
    return 0;
  };
  
  
}; // End of Master-IF module

#endif // AXI4_MASTER_IF_CON_H
