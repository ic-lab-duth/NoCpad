#ifndef WH_ROUTER_CON_CR_H
#define WH_ROUTER_CON_CR_H

#include <systemc.h>

#include "../../include/flit_axi.h"
#include "../../include/duth_fun.h"
#include "../../include/arbiters.h"
#include "../../fifo/fifo_queue_oh.h"

#include "nvhls_connections.h"

// Select In/Out Ports, the type of flit and the Routing Computation calculation function
// For RC_METHOD: 0-> direct rc 1-> lut, 2-> type, ... , 4-> LUT based routing
// IN_NUM    : Number of inputs
// OUT_NUM   : Number of inputs
// flit_t    : The networks flit type
// DIM_X      : X Dimension of a 2-D mesh network. Used in XY routing
// NODES      : All possible target nodes of the network. Used in LUT routing
// VCS        : Number of Virtual Channels
// BUFF_DEPTH : Input Buffer slots
// RC_METHOD : Routing Computation Algotrithm
//               - 0 : Direct RC
//               - 1 : Constant RC    (for mergers)
//               - 2 : Packet type RC (for distinct routing of Writes and reads)
//               - 3 : For single stage NoCs
//               - 4 : LUT based RC
//               - 5 : XY routing with merged RD/WR Req-Resp

// ARB_C      : The arbiter type. Eg MATRIX, ROUND_ROBIN
template< unsigned int IN_NUM, unsigned int OUT_NUM, typename flit_t, int DIM_X=0, int NODES=1, unsigned VCS=2, unsigned BUFF_DEPTH=3, unsigned RC_METHOD=3, arb_type arbiter_t=MATRIX >
SC_MODULE(rtr_vc) {
public:
  typedef sc_uint< nvhls::log2_ceil<VCS>::val > cr_t;
  
  sc_in_clk   clk;
  sc_in<bool> rst_n;
  
  sc_in< sc_uint<dnp::D_W> >  route_lut[NODES];
  sc_in< sc_uint<dnp::D_W> >  id_x{"id_x"};
  sc_in< sc_uint<dnp::D_W> >  id_y{"id_y"};
  
  // input channels
  Connections::In<flit_t> data_in[IN_NUM];
  Connections::Out<cr_t>  cr_out[IN_NUM];
  // output channels
  Connections::Out<flit_t> data_out[OUT_NUM];
  Connections::In<cr_t>    cr_in[OUT_NUM];
  
  // Internals
  fifo_queue<flit_t, BUFF_DEPTH>  fifo[IN_NUM][VCS];
  bool                            out_lock[IN_NUM][VCS];
  onehot<OUT_NUM>                 out_port_locked[IN_NUM][VCS];
  
  onehot<BUFF_DEPTH+1>        credits[OUT_NUM][VCS];
  onehot<VCS>                 out_available[OUT_NUM];
  arbiter<VCS   , arbiter_t>  arb_sa1[IN_NUM];
  arbiter<IN_NUM, arbiter_t>  arb_sa2[OUT_NUM];
  
  // Constructor
  SC_HAS_PROCESS(rtr_vc);
  
  rtr_vc(sc_module_name name_ = "rtr_vc")
    :
    sc_module(name_)
  {
    SC_THREAD(router_job);
    sensitive << clk.pos();
    async_reset_signal_is(rst_n, false);
  }
  
  
  void router_job() {
    flit_t       vc_hol_flit[IN_NUM][VCS];
    onehot<VCS>  sa1_grants[IN_NUM];
    
    flit_t flit_to_xbar[IN_NUM];
    
    // The request and grants of the Inputs/Outputs
    onehot<OUT_NUM> req_sa2_per_i[IN_NUM];
    onehot<IN_NUM>  req_sa2_per_o[OUT_NUM];
    
    onehot<IN_NUM>  gnt_sa2_per_o[OUT_NUM];
    onehot<OUT_NUM> gnt_sa2_per_i[IN_NUM];
    
    // Reset per input state
    #pragma hls_unroll yes
    per_i_rst:for (unsigned char i=0; i<IN_NUM; ++i) {
      data_in[i].Reset();
      cr_out[i].Reset();
      #pragma hls_unroll yes
      for(unsigned v=0; v<VCS; ++v) out_lock[i][v] = false;
    }
    // Reset per output state
    #pragma hls_unroll yes
    per_o_rst:for(unsigned char j=0; j<OUT_NUM; ++j) {
      data_out[j].Reset();
      cr_in[j].Reset();
      #pragma hls_unroll yes
      for(unsigned v=0; v<VCS; ++v) {
        out_available[j].val[v] = true;
        credits[j][v] = onehot<BUFF_DEPTH+1>(1<<BUFF_DEPTH);
      }
    }
    
    // Post Reset
    #pragma hls_pipeline_init_interval 1
    #pragma pipeline_stall_mode flush
    while(1) {
      wait();
  
      // UpStream Interface
      bool    data_val_in[IN_NUM];
      bool    data_rdy_out[IN_NUM];
      flit_t  data_data_in[IN_NUM];
      
      bool    cr_val_out[IN_NUM];
      bool    cr_rdy_in[IN_NUM];
      cr_t  cr_data_out[IN_NUM];
      // DowStream Interface
      bool    data_val_out[OUT_NUM];
      bool    data_rdy_in[OUT_NUM];
      flit_t  data_data_out[OUT_NUM];
  
      bool    cr_val_in[OUT_NUM];
      bool    cr_rdy_out[OUT_NUM];
      cr_t    cr_data_in[OUT_NUM];
  
      onehot<VCS> out_ready[OUT_NUM];
      
      // Read all inputs
      #pragma hls_unroll yes
      for (int i=0; i<IN_NUM; ++i) {
        data_val_in[i] = data_in[i].PopNB(data_data_in[i]);
      }
      
      #pragma hls_unroll yes
      for (int j=0; j<OUT_NUM; ++j) {
        cr_val_in[j] = cr_in[j].PopNB(cr_data_in[j]);
        
        // Check Credits
        #pragma hls_unroll yes
        for (int v = 0; v < VCS; ++v) {
          out_ready[j][v] = (credits[j][v].is_ready());
        }
      }
      
      // Input logic, loops for each input to produce the required requests
      #pragma hls_unroll yes
      input_prep : for (int i = 0; i < IN_NUM; ++i) {
        onehot<VCS>      req_sa1;
        onehot<OUT_NUM>  port_req_oh[VCS];
        
        // prepare requests of each VC, to content in SA1
        #pragma hls_unroll yes
        vc_prep : for (unsigned v=0; v<VCS; ++v) {
          vc_hol_flit[i][v] = fifo[i][v].peek();
          
          // Depending the Flit type the input selects an output port to request.
          // The required output gets stored to be used by the rest of the flits.
          if (out_lock[i][v]) {
            port_req_oh[v].set(out_port_locked[i][v]);
          } else {
            // Route Computation
            unsigned char current_op;
            if      (RC_METHOD==0) { current_op = do_rc_direct(vc_hol_flit[i][v].get_dst());} // returns the node ID
            else if (RC_METHOD==1) { current_op = do_rc_const();} // returns 0. Used for mergers
            else if (RC_METHOD==2) { current_op = do_rc_type(vc_hol_flit[i][v].get_type());} // Return 0/1 depending the type. used for splitters
            else if (RC_METHOD==3) { current_op = do_rc_common(vc_hol_flit[i][v].get_dst(),vc_hol_flit[i][v].get_type());}
            else if (RC_METHOD==4) { current_op = do_rc_lut(vc_hol_flit[i][v].get_dst());}
            else if (RC_METHOD==5) { current_op = do_rc_xy_merge(vc_hol_flit[i][v].get_dst(), vc_hol_flit[i][v].get_type());}
            else                   { NVHLS_ASSERT_MSG(0, "Wrong Routing method selected.");}
            
            port_req_oh[v].set(current_op);
            out_port_locked[i][v].set(port_req_oh[v]);
          }
          
          // The required output port must be also Ready and or available.
          onehot<VCS> req_out_ready_vcs = mux<onehot<VCS>, OUT_NUM>::mux_oh_case(port_req_oh[v], out_ready);
          onehot<VCS> req_out_avail_vcs = mux<onehot<VCS>, OUT_NUM>::mux_oh_case(port_req_oh[v], out_available);
          
          bool req_out_ready = req_out_ready_vcs[v];
          bool req_out_avail = req_out_avail_vcs[v];
  
          req_sa1[v] = (fifo[i][v].valid() && req_out_ready && (out_lock[i][v] || req_out_avail));
        }
        
        // Arbitrate amonng the VCs and select the winner to access SA2 and output MUX
        bool any_sa1_gnt = arb_sa1[i].arbitrate(req_sa1.val, sa1_grants[i].val);
        
        flit_to_xbar[i]  = mux<flit_t, VCS>::mux_oh_case(sa1_grants[i], vc_hol_flit[i]);
        req_sa2_per_i[i] = mux<onehot<OUT_NUM>, VCS>::mux_oh_case(sa1_grants[i], port_req_oh).and_mask(any_sa1_gnt);
      } // End of set inputs
  
      // Per Output arbitration and multiplexing
      #pragma hls_unroll yes
      outp_route : for (unsigned char j = 0; j < OUT_NUM; ++j) {
        // Swap from per input to per output
        #pragma hls_unroll yes
        for(int i=0; i<IN_NUM; ++i) {
          req_sa2_per_o[j][i] = req_sa2_per_i[i][j];
        }
        // SA2 arbitration among the inputs to win the output and the required VC
        bool any_gnt = arb_sa2[j].arbitrate(req_sa2_per_o[j].val, gnt_sa2_per_o[j].val);
        
        flit_t selected_flit = mux<flit_t, IN_NUM>::mux_oh_case(gnt_sa2_per_o[j], flit_to_xbar);
        cr_t   selected_vc   = selected_flit.get_vc();
        
        data_val_out[j]  = any_gnt;
        data_data_out[j] = selected_flit;
        
        #pragma hls_unroll yes
        for (unsigned v=0; v<VCS; ++v) {
          bool cr_upd_this_vc = cr_val_in[j] && (cr_data_in[j]==v);
          bool cr_cons_this_vc = any_gnt     && (selected_vc  ==v);
          if      ( cr_cons_this_vc && (!cr_upd_this_vc)) credits[j][v].decrease();
          else if (!cr_cons_this_vc && ( cr_upd_this_vc)) credits[j][v].increase();
        }
        
        if (any_gnt) {
          if (selected_flit.is_head()) out_available[j][selected_vc] = false;
          if (selected_flit.is_tail()) out_available[j][selected_vc] = true;
        }
      } // End per output
      
      
      // Loop through each input and VC to handle the case of actually winning the output
      #pragma hls_unroll yes
      inp_feedback : for (unsigned char i = 0; i < IN_NUM; ++i) {
        #pragma hls_unroll yes
        for(int j=0; j<IN_NUM; ++j) {
          gnt_sa2_per_i[i][j] = gnt_sa2_per_o[j][i];
        }
        
        // Handle Grants and incoming flits
        bool sa2_grant = gnt_sa2_per_i[i].or_reduce();
        cr_val_out[i]  = sa2_grant;
        bool got_new_flit = data_val_in[i];
        cr_t new_flit_vc  = data_data_in[i].get_vc();
        
        if (got_new_flit) fifo[i][new_flit_vc].push_no_count_incr(data_data_in[i]);
        
        // Update the FIFO and VC state
        cr_t vc_popped = 0;
        #pragma hls_unroll yes
        for (unsigned v=0; v<VCS; ++v) {
          bool this_vc_popped = sa2_grant && sa1_grants[i][v];
          if (this_vc_popped) {
            cr_data_out[i] = v;
            fifo[i][v].inc_pop_ptr();
            
            if      (vc_hol_flit[i][v].is_head()) out_lock[i][v] = true;
            else if (vc_hol_flit[i][v].is_tail()) out_lock[i][v] = false;
          }
          
          bool this_vc_pushed = got_new_flit && (new_flit_vc==v);
          fifo[i][v].set_count(this_vc_pushed, this_vc_popped);
        }
      }
      
      // Write to outputs
      #pragma hls_unroll yes
      for (int i=0; i<IN_NUM; ++i) {
        if(cr_val_out[i]) {
          bool dbg_push_ok = cr_out[i].PushNB(cr_data_out[i]);
          NVHLS_ASSERT_MSG(dbg_push_ok, "Push Credit DROP!!!");
        }
      }
      #pragma hls_unroll yes
      for (int j=0; j<OUT_NUM; ++j) {
        if (data_val_out[j]) {
          bool dbg_push_ok = data_out[j].PushNB(data_data_out[j]);
          NVHLS_ASSERT_MSG(dbg_push_ok, "Push Data DROP!!!");
        }
      }
      
    } // End of while(1)
  }; //end router_job_credits
  
  
  
  // Direct RC : The Dst Node is the Output port
  //inline unsigned char do_rc_direct (const unsigned char destination) {return destination;};
  inline unsigned char do_rc_direct (sc_uint<dnp::D_W> destination) {return destination.to_uint();};
  // TYPE RC : Used for splitter/mergers. Routes RD to Out==0, and WR to Out==1
  inline unsigned char do_rc_type   (sc_uint<dnp::T_W> type) {return (type==dnp::PACK_TYPE__RD_REQ || type==dnp::PACK_TYPE__RD_RESP) ? 0 : 1;};
  // Const RC : Used for mergers to always request port #0
  inline unsigned char do_rc_const  () {return 0;};
  // Const RC : Used for mergers to always request port #0
  inline unsigned char do_rc_common  (sc_uint<dnp::D_W> destination, sc_uint<dnp::T_W> type) {
    if      (type==dnp::PACK_TYPE__RD_REQ)  return destination.to_uint();
    else if (type==dnp::PACK_TYPE__RD_RESP) return destination.to_uint()-2;
    else if (type==dnp::PACK_TYPE__WR_REQ)  return destination.to_uint()+2;
    else                                    return destination.to_uint();
  };
  inline unsigned char do_rc_lut (sc_uint<dnp::D_W> destination) {
    return route_lut[destination.to_uint()].read();
  };
  
  inline unsigned char do_rc_xy_merge  (sc_uint<dnp::D_W> destination, sc_uint<dnp::T_W> type) {
    sc_uint<dnp::D_W> this_id_x = id_x.read();
    sc_uint<dnp::D_W> this_id_y = id_y.read();
    
    sc_uint<dnp::D_W> dst_x = destination % DIM_X;
    sc_uint<dnp::D_W> dst_y = destination / DIM_X;
    
    if (dst_x>this_id_x) {
      return 1;
    } else if (dst_x<this_id_x) {
      return 0;
    } else {
      if (dst_y>this_id_y) {
        return 3;
      } else if (dst_y<this_id_y) {
        return 2;
      } else {
        if      (type==dnp::PACK_TYPE__RD_REQ)  return 4;
        else if (type==dnp::PACK_TYPE__RD_RESP) return 4;
        else if (type==dnp::PACK_TYPE__WR_REQ)  return 5;
        else                                    return 5;
      }
    }
  };

}; // End of Module

#endif // WH_ROUTER_CON_CR_H
