// --------------------------------------------------------- //
//       SLAVE-IF Is where the SLAVE CONNECTS!!!!!           //
//                                                           //
// Aka. Master <-> Master-IF <-> NoC <-> Slave-IF <-> Slave  //
// --------------------------------------------------------- //


#ifndef AXI4_SLAVE_IF_CON_H
#define AXI4_SLAVE_IF_CON_H

#include "systemc.h"
#include "nvhls_connections.h"

#include "../include/flit_ace.h"
#include "../include/ace.h"
#include "../include/dnp_ace_v0.h"
#include "../include/duth_fun.h"

#define LOG_MAX_OUTS 8

// --- Helping Data structures --- //
struct rd_trans_info_t {
  sc_uint<dnp::S_W>  src; // source of transaction
  sc_uint<dnp::ace::ID_W> tid;
  sc_uint<dnp::ace::BU_W> burst;
  sc_uint<dnp::ace::SZ_W> size;
  sc_uint<dnp::ace::LE_W> len;
  sc_uint<dnp::ace::AP_W> addr_part; // Used for resizing at slave
  bool                    is_coherent;
  
  inline friend std::ostream& operator << ( std::ostream& os, const rd_trans_info_t& info ) {
    os <<"S: "<< info.src /*<<", D: "<< info.dst*/ <<", TID: "<< info.tid <<", Bu: "<< info.burst <<"Si: "<< info.size <<"Le: "<< info.len /*<<", Ticket: "<<info.reord_tct*/;
#ifdef SYSTEMC_INCLUDED
    os << std::dec << "@" << sc_time_stamp();
#else
    os << std::dec << "@" << "no-timed";
#endif
    return os;
  }

#ifdef SYSTEMC_INCLUDED
  // Only for SystemC
  inline friend void sc_trace(sc_trace_file* tf, const rd_trans_info_t& info, const std::string& name) {
    sc_trace(tf, info.src,   name + ".src");
    //sc_trace(tf, info.dst,   name + ".dst");
    sc_trace(tf, info.tid,   name + ".tid");
    sc_trace(tf, info.burst, name + ".burst");
    sc_trace(tf, info.size,  name + ".size");
    sc_trace(tf, info.len,   name + ".len");
    sc_trace(tf, info.is_coherent,   name + ".coherent");
    // Needed only when reordering is supported
    //sc_trace(tf, info.reord_tct,   name + ".ticket");
  }
#endif
};

// Info passed between packetizer and depacketizer to inform about new and finished transactions.
struct wr_trans_info_t {
  sc_uint<dnp::S_W>       src;
  sc_uint<dnp::ace::ID_W> tid;
  bool                    is_coherent;
  
  inline friend std::ostream& operator << ( std::ostream& os, const wr_trans_info_t& info ) {
    os <<"S: "<< info.src << info.tid;
#ifdef SYSTEMC_INCLUDED
    os << std::dec << "@" << sc_time_stamp();
#else
    os << std::dec << "@" << "no-timed";
#endif
    return os;
  }

#ifdef SYSTEMC_INCLUDED
  // Only for SystemC
  inline friend void sc_trace(sc_trace_file* tf, const wr_trans_info_t& info, const std::string& name) {
    sc_trace(tf, info.src,   name + ".src");
    sc_trace(tf, info.tid,   name + ".tid");
  }
#endif
};

// --- Slave IF --- //
// AXI Slave connects the independent AXI RD and WR cahnnels to the interface
// The interface gets the Request packets and independently reconstructs the AXI depending the Slave's attributes
// The Responses are getting packetized into seperate threads and are fed back to the network
// Thus Slave interface comprises of 4 distinct/parallel blocks WR/RD pack and WR/RD depack
template <typename cfg>
SC_MODULE(ace_slave_if) {
  typedef typename ace::ace5<axi::cfg::ace>  ace5_;
  typedef typename axi::AXI4_Encoding      enc_;
  
  typedef flit_dnp<cfg::RREQ_PHITS>   rreq_flit_t;
  typedef flit_dnp<cfg::RRESP_PHITS>  rresp_flit_t;
  typedef flit_dnp<cfg::WREQ_PHITS>   wreq_flit_t;
  typedef flit_dnp<cfg::WRESP_PHITS>  wresp_flit_t;
  
  typedef sc_uint< nvhls::log2_ceil<cfg::RRESP_PHITS>::val > cnt_phit_rresp_t;
  typedef sc_uint< nvhls::log2_ceil<cfg::WREQ_PHITS>::val >  cnt_phit_wreq_t;
  
  const unsigned char RD_S_SIZE = nvhls::log2_ceil<cfg::RD_LANES>::val;
  const unsigned char WR_S_SIZE = nvhls::log2_ceil<cfg::WR_LANES>::val;
  
  // The node ID, use for internal routing
  sc_in< sc_uint<dnp::D_W> > THIS_ID;
  
  sc_in_clk    clk;
  sc_in <bool> rst_n;
  
  // Memory Map, Slave's base address
  sc_in < sc_uint<(dnp::ace::AH_W+dnp::ace::AL_W)> > slave_base_addr;
  
  // NoC Side flit Channels
  Connections::In<rreq_flit_t>   INIT_S1(rd_flit_in);
  Connections::Out<rresp_flit_t> INIT_S1(rd_flit_out);
  
  Connections::In<wreq_flit_t>   INIT_S1(wr_flit_in);
  Connections::Out<wresp_flit_t> INIT_S1(wr_flit_out);
  
  // SLAVE Side AXI Channels
  // --- READ --- //
  Connections::Out<ace5_::AddrPayload>  INIT_S1(ar_out);
  Connections::In<ace5_::ReadPayload>   INIT_S1(r_in);
  
  // --- WRITE --- //
  Connections::Out<ace5_::AddrPayload>  INIT_S1(aw_out);
  Connections::Out<ace5_::WritePayload> INIT_S1(w_out);
  Connections::In<ace5_::WRespPayload>  INIT_S1(b_in);
  
  // --- READ Internal FIFOs --- //
  // FIFOs that pass initiation and finish transactions between Pack-Depack
  sc_fifo<rd_trans_info_t>           INIT_S1(rd_trans_init); // De-pack to Pack     | fwd to bck
  sc_fifo< sc_uint<dnp::ace::ID_W> > INIT_S1(rd_trans_fin);  // Pack    to De-pack  | bck to fwd
  
  // --- WRITE Internal FIFOs --- //
  sc_fifo<wr_trans_info_t>           INIT_S1(wr_trans_init); // Depack to pack
  sc_fifo< sc_uint<dnp::ace::ID_W> > INIT_S1(wr_trans_fin);  // Pack   to Depack
  
  // Constructor
  SC_HAS_PROCESS(ace_slave_if);
    
    ace_slave_if(sc_module_name name_="ace_slave_if")
    : 
    sc_module (name_),
    rd_trans_init (3),
    rd_trans_fin  (3),
    wr_trans_init (3),
    wr_trans_fin  (3)
  { 
    SC_THREAD(rd_req_depack_job);
    sensitive << clk.pos();
    async_reset_signal_is(rst_n, false);
  
    SC_THREAD(rd_resp_pack_job);
    sensitive << clk.pos();
    async_reset_signal_is(rst_n, false);
    
    
    SC_THREAD(wr_req_depack_job);
    sensitive << clk.pos();
    async_reset_signal_is(rst_n, false);
  
    SC_THREAD(wr_resp_pack_job);
    sensitive << clk.pos();
    async_reset_signal_is(rst_n, false);
  }
  
  //---------------------------------//
  //--- READ REQuest Depacketizer ---//
  //---------------------------------//
  void rd_req_depack_job () {
    sc_uint<LOG_MAX_OUTS>    rd_in_flight =  0; // Number of outstanding transactions
    sc_uint<dnp::ace::ID_W>  outst_tid    = -1; // TID of the last transaction
    rreq_flit_t              flit_rcv;
    
    ar_out.Reset();
    rd_flit_in.Reset();
    wait();
    while(1) {
      // Poll NoC for request flits
      if(rd_flit_in.PopNB(flit_rcv)) { // New request
        sc_uint<dnp::ace::ID_W> orig_tid = (flit_rcv.data[0] >> dnp::ace::req::ID_PTR) & ((1<<dnp::ace::ID_W)-1);
        sc_uint<dnp::S_W>  req_src  = (flit_rcv.data[0] >> dnp::S_PTR)       & ((1<<dnp::S_W)-1);
        
        // Wait while reordering is possible
        while ( ((rd_in_flight>0) && (orig_tid != outst_tid)) || (rd_in_flight>2) ) {
          // Check for finished transactions
          sc_uint<dnp::ace::ID_W> fin_tid;
          if(rd_trans_fin.nb_read(fin_tid)) {
            rd_in_flight--;
            NVHLS_ASSERT(fin_tid==outst_tid);
          }
          wait();
        };
        
        // --- Start of Request reconstruction ---
        // Get transaction info and check for resizing
        sc_uint<dnp::ace::SZ_W> init_size = (flit_rcv.data[2] >> dnp::ace::req::SZ_PTR) & ((1<<dnp::ace::SZ_W)-1);
        sc_uint<dnp::ace::LE_W> init_len  = (flit_rcv.data[1] >> dnp::ace::req::LE_PTR) & ((1<<dnp::ace::LE_W)-1);
        
        // In case of resizing the size and length changes in Slave's terms
        sc_uint<dnp::ace::SZ_W> final_size = (init_size>RD_S_SIZE) ? (sc_uint<dnp::ace::SZ_W>)RD_S_SIZE : init_size;
        sc_uint<dnp::ace::LE_W> final_len  = (init_size>RD_S_SIZE) ? (sc_uint<dnp::ace::LE_W>)(((init_len+1)<<(init_size-final_size))-1) : init_len;
        
        // Build the appropriate request for Slave
        ace5_::AddrPayload  temp_req;
        temp_req.id    = orig_tid.to_uint();
        temp_req.len   = final_len.to_uint();
        temp_req.size  = final_size.to_uint();
        temp_req.burst = (flit_rcv.data[2] >> dnp::ace::req::BU_PTR)  & ((1<<dnp::ace::BU_W)-1);
        temp_req.addr  = ((((flit_rcv.data[2]>>dnp::ace::req::AH_PTR) & ((1<<dnp::ace::AH_W)-1)) << dnp::ace::AL_W) |
                           ((flit_rcv.data[1]>>dnp::ace::req::AL_PTR) & ((1<<dnp::ace::AL_W)-1)))
                         - slave_base_addr.read();
        
        // Build the necessary info for Depacketizer
        rd_trans_info_t temp_info;
        temp_info.src         = (flit_rcv.data[0] >> dnp::S_PTR)   & ((1<<dnp::S_W)-1);
        temp_info.tid         = orig_tid;
        temp_info.len         = (flit_rcv.data[1] >> dnp::ace::req::LE_PTR) & ((1<<dnp::ace::LE_W)-1);
        temp_info.size        = (flit_rcv.data[2] >> dnp::ace::req::SZ_PTR) & ((1<<dnp::ace::SZ_W)-1);
        temp_info.burst       = (flit_rcv.data[2] >> dnp::ace::req::BU_PTR) & ((1<<dnp::ace::BU_W)-1);
        temp_info.addr_part   = (flit_rcv.data[1] & ((1<<dnp::ace::AP_W)-1));
        temp_info.is_coherent = (flit_rcv.get_type() > 3);
  
        NVHLS_ASSERT(((flit_rcv.data[0].to_uint() >> dnp::D_PTR) & ((1<<dnp::D_W)-1)) == (THIS_ID.read().to_uint()));
        
        rd_in_flight++;
        outst_tid = temp_info.tid;
        
        rd_trans_init.write(temp_info);
        ar_out.Push(temp_req);
      } else { // No new transaction, Check for finished transaction
        // Check for finished transactions
        sc_uint<dnp::ace::ID_W> fin_tid;
        if(rd_trans_fin.nb_read(fin_tid)) {
          rd_in_flight--;
          NVHLS_ASSERT(fin_tid==outst_tid);
        }
        wait();
      }
    } // End of while(1)
  }; // End of Read Request Packetizer
  
  //--------------------------------//
  //--- READ RESPonce Packetizer ---//
  //--------------------------------//
  void rd_resp_pack_job () {
    rd_flit_out.Reset();
    r_in.Reset();
    while(1) {
      rresp_flit_t temp_flit;
      
      // Blocking read. When Info is read for a transaction, a response is expected.
      // Thus, first the header is packed and sent to NoC
      // Then responce data are gathered, packed and again sent to NoC
      rd_trans_info_t this_head = rd_trans_init.read();
      sc_uint<dnp::T_W> packet_type = this_head.is_coherent ? dnp::PACK_TYPE__C_RD_RESP : dnp::PACK_TYPE__RD_RESP ;
      //--- Build header ---
      temp_flit.type    = HEAD;
      temp_flit.data[0] = ((sc_uint<dnp::PHIT_W>)this_head.burst             << dnp::ace::rresp::BU_PTR) |
                          ((sc_uint<dnp::PHIT_W>)this_head.tid               << dnp::ace::rresp::ID_PTR) |
                          ((sc_uint<dnp::PHIT_W>)packet_type              << dnp::T_PTR) |
                          ((sc_uint<dnp::PHIT_W>)0                        << dnp::Q_PTR) |
                          ((sc_uint<dnp::PHIT_W>)this_head.src               << dnp::D_PTR) |
                          ((sc_uint<dnp::PHIT_W>)THIS_ID                     << dnp::S_PTR) |
                          ((sc_uint<dnp::PHIT_W>)0                        << dnp::V_PTR) ;
      temp_flit.data[1] = ((sc_uint<dnp::PHIT_W>)(this_head.addr_part) << dnp::ace::rresp::AP_PTR) |
                          ((sc_uint<dnp::PHIT_W>)this_head.len         << dnp::ace::rresp::LE_PTR) |
                          ((sc_uint<dnp::PHIT_W>)this_head.size        << dnp::ace::rresp::SZ_PTR) ;
      
      rd_flit_out.Push(temp_flit); // Push Header flit to NoC
      
      // --- Start DATA Packetization --- //
      sc_uint<dnp::ace::SZ_W> final_size        = (this_head.size>RD_S_SIZE) ? (sc_uint<dnp::ace::SZ_W>) RD_S_SIZE : this_head.size;
      sc_uint<8>              addr_init_aligned = (this_head.addr_part & (cfg::RD_LANES-1)) & ~((1<<final_size)-1);
      
      // For data Depacketization we keep 2 pointers.
      //   - One to keep track axi byte lanes to place to data  (axi_lane_ptr)
      //   - One to point at the data of the flit               (flit_phit_ptr)
      sc_uint<8>        axi_lane_ptr  = addr_init_aligned;
      cnt_phit_rresp_t  flit_phit_ptr = 0;
  
      sc_uint<16> bytes_total  = ((this_head.len+1)<<this_head.size);  // Total number of bytes in the transaction
      sc_uint<16> bytes_packed = 0;                                    // Number of the packetized bytes
      
      unsigned char   data_build_tmp[cfg::WR_LANES];
      sc_uint<dnp::ace::R_RE_W> resp_tmp;
      sc_uint<dnp::ace::LA_W> last_tmp;
      
      #pragma hls_pipeline_init_interval 1
      #pragma pipeline_stall_mode flush
      gather_beats: while(1) {
        // Calculate the bytes to transfer in this iteration,
        //   depending the available flit bytes and the remaining to fill the beat
        sc_uint<8> bytes_axi_left  = ((1<<final_size) - (axi_lane_ptr & ((1<<final_size)-1)));
        sc_uint<8> bytes_flit_left = ((cfg::RRESP_PHITS<<1) - (flit_phit_ptr<<1));
        sc_uint<8> bytes_per_iter  = (bytes_axi_left<bytes_flit_left) ? bytes_axi_left : bytes_flit_left;
  
        // When the axi lane pointer wraps a size get the next beat
        if((bytes_packed & ((1<<final_size)-1))==0) {
          ace5_::ReadPayload this_resp;
          this_resp = r_in.Pop();
          duth_fun<ace5_::Data , cfg::RD_LANES>::assign_ac2char(data_build_tmp , this_resp.data);
          last_tmp = this_resp.last;
          resp_tmp = this_resp.resp;
        }
        
        // Convert AXI Beats to flits
        #pragma hls_unroll yes
        for (int i=0; i<cfg::RRESP_PHITS; ++i) { // i counts phits on the flit
          if(i>=flit_phit_ptr && i<(flit_phit_ptr+(bytes_per_iter>>1))) {
            sc_uint<8> loc_axi_ptr = (axi_lane_ptr + ((i-flit_phit_ptr)<<1));
            temp_flit.data[i] = ((sc_uint<dnp::PHIT_W>)resp_tmp                      << dnp::ace::rdata::RE_PTR) | // MSB
                                ((sc_uint<dnp::PHIT_W>)last_tmp                      << dnp::ace::rdata::LA_PTR) |
                                ((sc_uint<dnp::PHIT_W>)data_build_tmp[loc_axi_ptr+1] << dnp::ace::rdata::B1_PTR) | // (i*2) % 4
                                ((sc_uint<dnp::PHIT_W>)data_build_tmp[loc_axi_ptr  ] << dnp::ace::rdata::B0_PTR) ; // LSB
          }
        }
        
        // transaction event flags 
        bool done_job  = ((bytes_packed+bytes_per_iter)==bytes_total);             // All bytes are processed
        bool done_flit = (flit_phit_ptr+(bytes_per_iter>>1)==cfg::RRESP_PHITS);    // Flit got empty
        bool done_axi  = (((bytes_packed+bytes_per_iter)&((1<<final_size)-1))==0); // Beat got full
        
        // Push the flit to NoC
        if(done_job || done_flit) {
          temp_flit.type = (done_job) ? TAIL : BODY;
          rd_flit_out.Push(temp_flit);
        }
        
        if (done_job) { // End of transaction
          bytes_packed = 0;
          rd_trans_fin.write(this_head.tid);
          break;
        } else {  // Move to next iteration
          bytes_packed  += bytes_per_iter;
          flit_phit_ptr = (done_flit) ? 0 : (flit_phit_ptr +(bytes_per_iter>>1));
          axi_lane_ptr  = (this_head.burst==enc_::AXBURST::FIXED) ? ((axi_lane_ptr+bytes_per_iter) & ((1<<final_size)-1)) + addr_init_aligned :
                                                                    ((axi_lane_ptr+bytes_per_iter) & (cfg::RD_LANES-1)) ;
        }
      } // End of transaction loop
    } // End of While(1)
  }; // End of Read Responce Packetizer
  
  
  //-----------------------------------//
  //--- WRITE REQuest DE-Packetizer ---//
  //-----------------------------------//  
  void wr_req_depack_job () {
    sc_uint<LOG_MAX_OUTS>    wr_in_flight = 0;
    sc_uint<dnp::ace::ID_W>  outst_tid    = -1;
    
    aw_out.Reset();
    w_out.Reset();
    wr_flit_in.Reset();
    while(1) {
      wreq_flit_t   flit_rcv;
      if (wr_flit_in.PopNB(flit_rcv)) {
        sc_uint<dnp::ace::ID_W> orig_tid = (flit_rcv.data[0] >> dnp::ace::req::ID_PTR) & ((1<<dnp::ace::ID_W)-1);
        sc_uint<dnp::S_W>  req_src  = (flit_rcv.data[0] >> dnp::S_PTR)       & ((1<<dnp::S_W)-1);
        
        // Wait while reordering is possible
        while ( ((wr_in_flight>0) && (orig_tid != outst_tid)) || (wr_in_flight>2) ) {
          // Check for finished transactions
          sc_uint<dnp::ace::ID_W> fin_tid;
          if(wr_trans_fin.nb_read(fin_tid)) {
            wr_in_flight--;
            NVHLS_ASSERT(fin_tid==outst_tid);
          }
          wait();
        };
  
        // --- Start of Request reconstruction ---
        // Get transaction info and check for resizing
        sc_uint<dnp::ace::SZ_W> init_size = (flit_rcv.data[2] >> dnp::ace::req::SZ_PTR) & ((1<<dnp::ace::SZ_W)-1);
        sc_uint<dnp::ace::LE_W> init_len  = (flit_rcv.data[1] >> dnp::ace::req::LE_PTR) & ((1<<dnp::ace::LE_W)-1);
        // In case of resizing the size and length changes in Slave's terms
        sc_uint<dnp::ace::SZ_W> final_size = (init_size>WR_S_SIZE) ? (sc_uint<dnp::ace::SZ_W>)WR_S_SIZE : init_size;
        sc_uint<dnp::ace::LE_W> final_len  = (init_size>WR_S_SIZE) ? (sc_uint<dnp::ace::LE_W>)(((init_len+1)<<(init_size-final_size))-1) : init_len;
        
        // Build the appropriate request for Slave
        ace5_::AddrPayload  this_req;
        this_req.id    = orig_tid.to_uint();
        this_req.len   = final_len.to_uint();
        this_req.size  = final_size.to_uint();
        this_req.burst = (flit_rcv.data[2] >> dnp::ace::req::BU_PTR)  & ((1<<dnp::ace::BU_W)-1);
        this_req.addr  = ((((flit_rcv.data[2]>>dnp::ace::req::AH_PTR) & ((1<<dnp::ace::AH_W)-1)) << dnp::ace::AL_W) |
                          ((flit_rcv.data[1]>>dnp::ace::req::AL_PTR)  & ((1<<dnp::ace::AL_W)-1)))
                         - slave_base_addr.read();
        
        NVHLS_ASSERT(((flit_rcv.data[0].to_uint() >> dnp::D_PTR) & ((1<<dnp::D_W)-1)) == (THIS_ID.read().to_uint()));
        
        // Build the necessary info for Packetizer
        wr_trans_info_t this_info;
        this_info.tid = orig_tid;
        this_info.src = req_src;
        this_info.is_coherent = (flit_rcv.get_type() > 3);
        
        // update bookkeeping vars
        wr_in_flight++;
        outst_tid = orig_tid;
        // Push info to Resp-pack and request to Slave
        wr_trans_init.write(this_info);
        aw_out.Push(this_req);
        
        unsigned char data_build_tmp[cfg::WR_LANES];
        bool          wstr_build_tmp[cfg::WR_LANES];
        #pragma hls_unroll yes
        for (int i=0; i<cfg::WR_LANES; ++i) {
          wstr_build_tmp[i] = false;
          data_build_tmp[i] = 0;
        }
        
        // Gather DATA
        sc_uint<8>        addr_init_aligned  = (this_req.addr.to_uint() & (cfg::WR_LANES-1)) & ~((1<<final_size)-1);
        sc_uint<8>        axi_lane_ptr       = addr_init_aligned;
        cnt_phit_wreq_t   flit_phit_ptr      = 0;
        
        sc_uint<16> bytes_total    = ((this_req.len.to_uint()+1)<<this_req.size.to_uint());
        sc_uint<16> bytes_depacked = 0;
        
        #pragma hls_pipeline_init_interval 1
        #pragma pipeline_stall_mode flush
        gather_wr_flits : while (1) {
          // Calculate the bytes to transfer in this iteration,
          //   depending the available flit bytes and the remaining to fill the beat
          sc_uint<8> bytes_axi_left  = ((1<<this_req.size.to_uint()) - (axi_lane_ptr & ((1<<this_req.size.to_uint())-1)));
          sc_uint<8> bytes_flit_left = ((cfg::WREQ_PHITS<<1)              - (flit_phit_ptr<<1));
          sc_uint<8> bytes_per_iter  = (bytes_axi_left<bytes_flit_left) ? bytes_axi_left : bytes_flit_left;
  
          // When the phit pointer resets get the flit
          if(flit_phit_ptr==0) {
            while (!wr_flit_in.PopNB(flit_rcv)) wait(); //flit_rcv = wr_flit_in.Pop();
          }
  
          // Convert AXI Beats to flits
          #pragma hls_unroll yes
          build_resp: for (unsigned int i=0; i<(cfg::WR_LANES>>1); ++i){ // i counts PHITS
            if(i>=(axi_lane_ptr>>1) && i<((axi_lane_ptr+bytes_per_iter)>>1)) {
              sc_uint<8> loc_flit_ptr = flit_phit_ptr + (i-(axi_lane_ptr>>1));
              data_build_tmp[(i<<1)+1] = (flit_rcv.data[loc_flit_ptr] >> dnp::ace::wdata::B1_PTR) & ((1<<dnp::ace::B_W)-1); // MSB
              data_build_tmp[(i<<1)  ] = (flit_rcv.data[loc_flit_ptr] >> dnp::ace::wdata::B0_PTR) & ((1<<dnp::ace::B_W)-1); // LSB
        
              wstr_build_tmp[(i<<1)+1] = (flit_rcv.data[loc_flit_ptr] >> dnp::ace::wdata::E1_PTR) & ((1<<dnp::ace::E_W)-1); // MSB
              wstr_build_tmp[(i<<1)  ] = (flit_rcv.data[loc_flit_ptr] >> dnp::ace::wdata::E0_PTR) & ((1<<dnp::ace::E_W)-1); // LSB
            }
          }
  
          // transaction event flags
          bool done_job  = ((bytes_depacked+bytes_per_iter)==bytes_total);             // All bytes are processed
          bool done_flit = (flit_phit_ptr+(bytes_per_iter>>1)==cfg::WREQ_PHITS);       // Flit got empty
          bool done_axi  = (((bytes_depacked+bytes_per_iter)&((1<<final_size)-1))==0); // Beat got full
  
          // Push the beat to slave when either this Beat got the needed bytes or all bytes are transferred
          if(done_job || done_axi ) {
            ace5_::WritePayload  builder_wr_data;
            //builder_wr_data.id   = this_req.id.to_uint(); // Deprecated in AXI4
            builder_wr_data.last = ((bytes_depacked+bytes_per_iter)==bytes_total);
            duth_fun<ace5_::Data , cfg::WR_LANES>::assign_char2ac(builder_wr_data.data , data_build_tmp);
            duth_fun<ace5_::Wstrb, cfg::WR_LANES>::assign_bool2ac(builder_wr_data.wstrb, wstr_build_tmp);
            w_out.Push(builder_wr_data);
            #pragma hls_unroll yes
            for (int i=0; i<cfg::WR_LANES; ++i) {
              wstr_build_tmp[i] = false;
              data_build_tmp[i] = 0;
            }
          }
          
          // Check to either finish transaction or update the pointers for the next iteration
          if (done_job) { // All data received
            break;
          } else {                                      // Still Building ...
            bytes_depacked +=bytes_per_iter;
            flit_phit_ptr = (done_flit) ? 0 : (flit_phit_ptr +(bytes_per_iter>>1));
            axi_lane_ptr   = ((unsigned)this_req.burst==enc_::AXBURST::FIXED) ? ((axi_lane_ptr+bytes_per_iter) & ((1<<this_req.size.to_uint())-1)) + addr_init_aligned :
                                                                      ((axi_lane_ptr+bytes_per_iter) & (cfg::WR_LANES-1)) ;
          }
        } // End of flit gather
      } else {
        // Check for finished transactions
        sc_uint<dnp::ace::ID_W> fin_tid;
        if(wr_trans_fin.nb_read(fin_tid)) {
          wr_in_flight--;
          NVHLS_ASSERT(fin_tid==outst_tid);
        }
        wait();
      }
    } // End of while(1)
    
  }; // End of Write Request DE-Packetizer
  
  //---------------------------------//
  //--- WRITE RESPonce Packetizer ---//
  //---------------------------------//
  void wr_resp_pack_job(){
    wr_flit_out.Reset();
    b_in.Reset();
    while(1) {
      wait();
      wresp_flit_t temp_flit;
      
      // Blocking read. First get head info, then get WR responce  and pack it.
      wr_trans_info_t this_head = wr_trans_init.read();
      ace5_::WRespPayload this_resp = b_in.Pop();
  
      sc_uint<dnp::T_W> packet_type = this_head.is_coherent ? dnp::PACK_TYPE__C_WR_RESP : dnp::PACK_TYPE__WR_RESP ;
      
      temp_flit.type = SINGLE;
      temp_flit.data[0] = ((sc_uint<dnp::PHIT_W>)this_resp.resp             << dnp::ace::wresp::RESP_PTR ) |
                          ((sc_uint<dnp::PHIT_W>)this_head.tid              << dnp::ace::wresp::ID_PTR )   |
                          ((sc_uint<dnp::PHIT_W>)packet_type             << dnp::T_PTR ) |
                          ((sc_uint<dnp::PHIT_W>)0                       << dnp::Q_PTR ) |
                          ((sc_uint<dnp::PHIT_W>)this_head.src              << dnp::D_PTR ) |
                          ((sc_uint<dnp::PHIT_W>)THIS_ID                    << dnp::S_PTR ) ;
      
      wr_flit_out.Push(temp_flit);
      wr_trans_fin.write(this_head.tid);
    } // End of While(1)
  }; // End of Write Resp Packetizer
  

}; // End of Slave-IF module

#endif // AXI4_SLAVE_IF_CON_H
