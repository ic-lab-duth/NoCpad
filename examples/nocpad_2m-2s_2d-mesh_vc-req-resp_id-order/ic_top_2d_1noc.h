#ifndef AXI4_TOP_IC_H
#define AXI4_TOP_IC_H

#pragma once

#include "../../src/axi_ifs/vc_based/axi_master_if_vc.h"
#include "../../src/axi_ifs/vc_based/axi_slave_if_vc.h"

#include "../../src/routers/router_vc.h"

#include "systemc.h"
#include "nvhls_connections.h"

#pragma hls_design top

// Bundle of configuration parameters
template <
  unsigned char MASTER_NUM_ , unsigned char SLAVE_NUM_,
  unsigned char RD_LANES_   , unsigned char WR_LANES_,
  unsigned char RREQ_PHITS_ , unsigned char RRESP_PHITS_,
  unsigned char WREQ_PHITS_ , unsigned char WRESP_PHITS_,
  unsigned char ORD_SCHEME_ , unsigned char VCS_
>
struct cfg {
  static const unsigned char MASTER_NUM  = MASTER_NUM_;
  static const unsigned char SLAVE_NUM   = SLAVE_NUM_;
  static const unsigned char RD_LANES    = RD_LANES_;
  static const unsigned char WR_LANES    = WR_LANES_;
  static const unsigned char RREQ_PHITS  = RREQ_PHITS_;
  static const unsigned char RRESP_PHITS = RRESP_PHITS_;
  static const unsigned char WREQ_PHITS  = WREQ_PHITS_;
  static const unsigned char WRESP_PHITS = WRESP_PHITS_;
  static const unsigned char ORD_SCHEME  = ORD_SCHEME_;
  static const unsigned char VCS  = VCS_;
};

// the used configuration. 2 Masters/Slaves, 64bit AXI, 2.4.4.1 phit flits
typedef cfg<2, 2, 8, 8, 4, 4, 4, 4, 1, 2> smpl_cfg;

SC_MODULE(ic_top) {
public:
  // typedef matchlib's axi with the "standard" configuration
  typedef typename axi::axi4<axi::cfg::standard_duth> axi4_;
  
  // typedef the 4 kind of flits(RD/WR Req/Resp) depending their size
  typedef flit_dnp<smpl_cfg::RREQ_PHITS>  rreq_flit_t;
  typedef flit_dnp<smpl_cfg::RRESP_PHITS> rresp_flit_t;
  typedef flit_dnp<smpl_cfg::WREQ_PHITS>  wreq_flit_t;
  typedef flit_dnp<smpl_cfg::WRESP_PHITS> wresp_flit_t;
    
  typedef sc_uint<1> cr_t;
    
  static const unsigned DIM_X = 2;
  static const unsigned DIM_Y = 2;
  
  sc_in_clk    clk;
  sc_in <bool> rst_n;
  
  // IC's Address map
  sc_in<sc_uint <32> >           addr_map[smpl_cfg::SLAVE_NUM][2]; // [SLAVE_NUM][0:begin, 1: End]
  
  sc_signal< sc_uint<dnp::D_W> >  route_lut[2][1];
  
  // The Node IDs are passed to IFs as signals
  sc_signal< sc_uint<dnp::S_W> > NODE_IDS_MASTER[smpl_cfg::MASTER_NUM];
  sc_signal< sc_uint<dnp::S_W> > NODE_IDS_SLAVE[smpl_cfg::SLAVE_NUM];
  
  sc_signal< sc_uint<dnp::D_W> > rtr_id_x_req[DIM_X];
  sc_signal< sc_uint<dnp::D_W> > rtr_id_y_req[DIM_Y];
  
  sc_signal< sc_uint<dnp::D_W> > rtr_id_x_resp[DIM_X];
  sc_signal< sc_uint<dnp::D_W> > rtr_id_y_resp[DIM_Y];
  
  // MASTER Side AXI Channels
  Connections::In<axi4_::AddrPayload>   ar_in[smpl_cfg::MASTER_NUM];
  Connections::Out<axi4_::ReadPayload>  r_out[smpl_cfg::MASTER_NUM];
  
  Connections::In<axi4_::AddrPayload>   aw_in[smpl_cfg::MASTER_NUM];
  Connections::In<axi4_::WritePayload>  w_in[smpl_cfg::MASTER_NUM];
  Connections::Out<axi4_::WRespPayload> b_out[smpl_cfg::MASTER_NUM];
  
  // SLAVE Side AXI Channels
  Connections::Out<axi4_::AddrPayload>  ar_out[smpl_cfg::SLAVE_NUM];
  Connections::In<axi4_::ReadPayload>   r_in[smpl_cfg::SLAVE_NUM];
  
  Connections::Out<axi4_::AddrPayload>  aw_out[smpl_cfg::SLAVE_NUM];
  Connections::Out<axi4_::WritePayload> w_out[smpl_cfg::SLAVE_NUM];
  Connections::In<axi4_::WRespPayload>  b_in[smpl_cfg::SLAVE_NUM];
  
  //--- Internals ---//
  // --- Master/Slave IFs ---
  axi_master_if_vc < smpl_cfg > *master_if[smpl_cfg::MASTER_NUM];
  axi_slave_if_vc  < smpl_cfg > *slave_if[smpl_cfg::SLAVE_NUM];
  
  // Master IF Channels
  // READ Fwd Req
  Connections::Buffer<rreq_flit_t, 4>       rreq_m_buf_data[smpl_cfg::MASTER_NUM];
  Connections::Buffer<cr_t , 4>             rreq_m_buf_cr[smpl_cfg::MASTER_NUM];
  Connections::Combinational<rreq_flit_t>   rreq_m_chan_data[smpl_cfg::MASTER_NUM];
  Connections::Combinational<cr_t>          rreq_m_chan_cr[smpl_cfg::MASTER_NUM];
  Connections::Combinational<rreq_flit_t>   chan_rd_m2r_data[smpl_cfg::MASTER_NUM]; // M-IF_to_Rtr
  Connections::Combinational<cr_t>          chan_rd_m2r_cr[smpl_cfg::MASTER_NUM];   // M-IF_from_Rtr
  // READ Backward Resp
  Connections::Combinational<rresp_flit_t>   chan_rd_r2m_data[smpl_cfg::MASTER_NUM]; // Rtr_to_S-IF
  Connections::Combinational<cr_t>           chan_rd_r2m_cr[smpl_cfg::MASTER_NUM];   // Rtr_from_S-IF
  Connections::Buffer<rresp_flit_t, 4>       rresp_m_buf_data[smpl_cfg::MASTER_NUM];
  Connections::Buffer<cr_t , 4>              rresp_m_buf_cr[smpl_cfg::MASTER_NUM];
  Connections::Combinational<rresp_flit_t>   rresp_m_chan_data[smpl_cfg::MASTER_NUM];
  Connections::Combinational<cr_t>           rresp_m_chan_cr[smpl_cfg::MASTER_NUM];
  // WRITE Fwd Req
  Connections::Buffer<wreq_flit_t, 4>       wreq_m_buf_data[smpl_cfg::MASTER_NUM];
  Connections::Buffer<cr_t , 4>             wreq_m_buf_cr[smpl_cfg::MASTER_NUM];
  Connections::Combinational<wreq_flit_t>   wreq_m_chan_data[smpl_cfg::MASTER_NUM];
  Connections::Combinational<cr_t>          wreq_m_chan_cr[smpl_cfg::MASTER_NUM];
  Connections::Combinational<wreq_flit_t>   chan_wr_m2r_data[smpl_cfg::MASTER_NUM]; // M-IF_to_Rtr
  Connections::Combinational<cr_t>          chan_wr_m2r_cr[smpl_cfg::MASTER_NUM];   // M-IF_from_Rtr
  // WRITE Backward Resp
  Connections::Combinational<wresp_flit_t>   chan_wr_r2m_data[smpl_cfg::MASTER_NUM]; // Rtr_to_S-IF
  Connections::Combinational<cr_t>           chan_wr_r2m_cr[smpl_cfg::MASTER_NUM];   // Rtr_from_S-IF
  Connections::Buffer<wresp_flit_t, 4>       wresp_m_buf_data[smpl_cfg::MASTER_NUM];
  Connections::Buffer<cr_t , 4>              wresp_m_buf_cr[smpl_cfg::MASTER_NUM];
  Connections::Combinational<wresp_flit_t>   wresp_m_chan_data[smpl_cfg::MASTER_NUM];
  Connections::Combinational<cr_t>           wresp_m_chan_cr[smpl_cfg::MASTER_NUM];
  
  // Slave IF Channels
  // READ Fwd Req
  Connections::Combinational<rreq_flit_t>   chan_rd_r2s_data[smpl_cfg::SLAVE_NUM]; // Rtr_to_S-IF
  Connections::Combinational<cr_t>          chan_rd_r2s_cr[smpl_cfg::SLAVE_NUM];   // Rtr_from_S-IF
  Connections::Buffer<rreq_flit_t, 4>       rreq_s_buf_data[smpl_cfg::SLAVE_NUM];
  Connections::Buffer<cr_t , 4>             rreq_s_buf_cr[smpl_cfg::SLAVE_NUM];
  Connections::Combinational<rreq_flit_t>   rreq_s_chan_data[smpl_cfg::SLAVE_NUM];
  Connections::Combinational<cr_t>          rreq_s_chan_cr[smpl_cfg::SLAVE_NUM];
  // READ Backward Resp
  Connections::Buffer<rresp_flit_t, 4>       rresp_s_buf_data[smpl_cfg::SLAVE_NUM];
  Connections::Buffer<cr_t , 4>              rresp_s_buf_cr[smpl_cfg::SLAVE_NUM];
  Connections::Combinational<rresp_flit_t>   rresp_s_chan_data[smpl_cfg::SLAVE_NUM];
  Connections::Combinational<cr_t>           rresp_s_chan_cr[smpl_cfg::SLAVE_NUM];
  Connections::Combinational<rresp_flit_t>   chan_rd_s2r_data[smpl_cfg::SLAVE_NUM]; // M-IF_to_Rtr
  Connections::Combinational<cr_t>           chan_rd_s2r_cr[smpl_cfg::SLAVE_NUM];   // M-IF_from_Rtr
  // WRITE Fwd Req
  Connections::Combinational<wreq_flit_t>   chan_wr_r2s_data[smpl_cfg::SLAVE_NUM]; // Rtr_to_S-IF
  Connections::Combinational<cr_t>          chan_wr_r2s_cr[smpl_cfg::SLAVE_NUM];   // Rtr_from_S-IF
  Connections::Buffer<wreq_flit_t, 4>       wreq_s_buf_data[smpl_cfg::SLAVE_NUM];
  Connections::Buffer<cr_t , 4>             wreq_s_buf_cr[smpl_cfg::SLAVE_NUM];
  Connections::Combinational<wreq_flit_t>   wreq_s_chan_data[smpl_cfg::SLAVE_NUM];
  Connections::Combinational<cr_t>          wreq_s_chan_cr[smpl_cfg::SLAVE_NUM];
  // WRITE Backward Resp
  Connections::Buffer<wresp_flit_t, 4>       wresp_s_buf_data[smpl_cfg::SLAVE_NUM];
  Connections::Buffer<cr_t , 4>              wresp_s_buf_cr[smpl_cfg::SLAVE_NUM];
  Connections::Combinational<wresp_flit_t>   wresp_s_chan_data[smpl_cfg::SLAVE_NUM];
  Connections::Combinational<cr_t>           wresp_s_chan_cr[smpl_cfg::SLAVE_NUM];
  Connections::Combinational<wresp_flit_t>   chan_wr_s2r_data[smpl_cfg::SLAVE_NUM]; // M-IF_to_Rtr
  Connections::Combinational<cr_t>           chan_wr_s2r_cr[smpl_cfg::SLAVE_NUM];   // M-IF_from_Rtr
  
  
  
  // Master IF Channels
  // Read Req/Resp
  Connections::Combinational<rreq_flit_t>    chan_rd_m2r[smpl_cfg::MASTER_NUM];
  Connections::Combinational<rresp_flit_t>   chan_rd_r2m[smpl_cfg::MASTER_NUM];
  // Write Req/Resp
  Connections::Combinational<wreq_flit_t>    chan_wr_m2r[smpl_cfg::MASTER_NUM];
  Connections::Combinational<wresp_flit_t>   chan_wr_r2m[smpl_cfg::MASTER_NUM];
  
  // Slave IF
  // Read Req/Resp
  Connections::Combinational<rreq_flit_t>    chan_rd_r2s[smpl_cfg::SLAVE_NUM];
  Connections::Combinational<rresp_flit_t>   chan_rd_s2r[smpl_cfg::SLAVE_NUM];
  
  Connections::Combinational<wreq_flit_t>    chan_wr_r2s[smpl_cfg::SLAVE_NUM];
  Connections::Combinational<wresp_flit_t>   chan_wr_s2r[smpl_cfg::SLAVE_NUM];
  
  // --- NoC Channels ---
  // REQ Router + In/Out Channels
  rtr_vc< 4+2, 4+2, rreq_flit_t, DIM_X, 1, 2, 3, 5>   rtr_inst[DIM_X][DIM_Y];
  
  Connections::Combinational<rreq_flit_t>    chan_hor_right_data[DIM_X+1][DIM_Y];
  Connections::Combinational<cr_t>           chan_hor_right_cr[DIM_X+1][DIM_Y];
  Connections::Combinational<rreq_flit_t>    chan_hor_left_data[DIM_X+1][DIM_Y];
  Connections::Combinational<cr_t>           chan_hor_left_cr[DIM_X+1][DIM_Y];
  
  Connections::Combinational<rreq_flit_t>    chan_ver_up_data[DIM_X][DIM_Y+1];
  Connections::Combinational<cr_t>           chan_ver_up_cr[DIM_X][DIM_Y+1];
  Connections::Combinational<rreq_flit_t>    chan_ver_down_data[DIM_X][DIM_Y+1];
  Connections::Combinational<cr_t>           chan_ver_down_cr[DIM_X][DIM_Y+1];
  
  Connections::Combinational<rreq_flit_t>    chan_inj_wr_data[DIM_X][DIM_Y];
  Connections::Combinational<cr_t>           chan_inj_wr_cr[DIM_X][DIM_Y];
  Connections::Combinational<rreq_flit_t>    chan_inj_rd_data[DIM_X][DIM_Y];
  Connections::Combinational<cr_t>           chan_inj_rd_cr[DIM_X][DIM_Y];
  
  Connections::Combinational<rreq_flit_t>    chan_ej_wr_data[DIM_X][DIM_Y];
  Connections::Combinational<cr_t>           chan_ej_wr_cr[DIM_X][DIM_Y];
  Connections::Combinational<rreq_flit_t>    chan_ej_rd_data[DIM_X][DIM_Y];
  Connections::Combinational<cr_t>           chan_ej_rd_cr[DIM_X][DIM_Y];
  
  
  SC_CTOR(ic_top) {
    
    route_lut[0][0] = 0;
    route_lut[1][0] = 0;
    
    // ----------------- //
    // --- SLAVE-IFs --- //
    // ----------------- //
    for(unsigned char j=0; j<smpl_cfg::SLAVE_NUM; ++j){
      NODE_IDS_SLAVE[j] = j;
      
      unsigned col = j % DIM_X; // aka x dim
      unsigned row = j / DIM_X; // aka y dim
      
      slave_if[j] = new axi_slave_if_vc < smpl_cfg > (sc_gen_unique_name("Slave-if"));
      slave_if[j]->clk(clk);
      slave_if[j]->rst_n(rst_n);
      
      slave_if[j]->THIS_ID(NODE_IDS_SLAVE[j]);
      slave_if[j]->slave_base_addr(addr_map[j][0]);
      
      // Read-NoC
      slave_if[j]->rd_flit_data_in(rreq_s_chan_data[j]);
      slave_if[j]->rd_flit_cr_out(rreq_s_chan_cr[j]);
      slave_if[j]->rd_flit_data_out(rresp_s_chan_data[j]);
      slave_if[j]->rd_flit_cr_in(rresp_s_chan_cr[j]);
      // Write-NoC
      slave_if[j]->wr_flit_data_in(wreq_s_chan_data[j]);
      slave_if[j]->wr_flit_cr_out(wreq_s_chan_cr[j]);
      slave_if[j]->wr_flit_data_out(wresp_s_chan_data[j]);
      slave_if[j]->wr_flit_cr_in(wresp_s_chan_cr[j]);
      
      // Slave-Side
      slave_if[j]->ar_out(ar_out[j]);
      slave_if[j]->r_in(r_in[j]);
      
      slave_if[j]->aw_out(aw_out[j]);
      slave_if[j]->w_out(w_out[j]);
      slave_if[j]->b_in(b_in[j]);
  
      // Buffer Connectivity
      // Read Request
      rreq_s_buf_data[j].clk(clk);
      rreq_s_buf_data[j].rst(rst_n);
      rreq_s_buf_data[j].enq(chan_ej_rd_data[col][row]);
      rreq_s_buf_data[j].deq(rreq_s_chan_data[j]);
  
      rreq_s_buf_cr[j].clk(clk);
      rreq_s_buf_cr[j].rst(rst_n);
      rreq_s_buf_cr[j].enq(rreq_s_chan_cr[j]);
      rreq_s_buf_cr[j].deq(chan_ej_rd_cr[col][row]);
  
      // Read Response
      rresp_s_buf_data[j].clk(clk);
      rresp_s_buf_data[j].rst(rst_n);
      rresp_s_buf_data[j].enq(rresp_s_chan_data[j]);
      rresp_s_buf_data[j].deq(chan_inj_rd_data[col][row]);
  
      rresp_s_buf_cr[j].clk(clk);
      rresp_s_buf_cr[j].rst(rst_n);
      rresp_s_buf_cr[j].enq(chan_inj_rd_cr[col][row]);
      rresp_s_buf_cr[j].deq(rresp_s_chan_cr[j]);
  
      // Write Request
      wreq_s_buf_data[j].clk(clk);
      wreq_s_buf_data[j].rst(rst_n);
      wreq_s_buf_data[j].enq(chan_ej_wr_data[col][row]);
      wreq_s_buf_data[j].deq(wreq_s_chan_data[j]);
  
      wreq_s_buf_cr[j].clk(clk);
      wreq_s_buf_cr[j].rst(rst_n);
      wreq_s_buf_cr[j].enq(wreq_s_chan_cr[j]);
      wreq_s_buf_cr[j].deq(chan_ej_wr_cr[col][row]);
  
      // Write Response
      wresp_s_buf_data[j].clk(clk);
      wresp_s_buf_data[j].rst(rst_n);
      wresp_s_buf_data[j].enq(wresp_s_chan_data[j]);
      wresp_s_buf_data[j].deq(chan_inj_wr_data[col][row]);
  
      wresp_s_buf_cr[j].clk(clk);
      wresp_s_buf_cr[j].rst(rst_n);
      wresp_s_buf_cr[j].enq(chan_inj_wr_cr[col][row]);
      wresp_s_buf_cr[j].deq(wresp_s_chan_cr[j]);
    }
    
    // ------------------------------ //
    // --- MASTER-IFs Connectivity--- //
    // ------------------------------ //
    for (int i=0; i<smpl_cfg::MASTER_NUM; ++i) {
      NODE_IDS_MASTER[i] = smpl_cfg::SLAVE_NUM + i;
  
      unsigned col = (smpl_cfg::SLAVE_NUM + i) % DIM_X; // aka x dim
      unsigned row = (smpl_cfg::SLAVE_NUM + i) / DIM_X; // aka y dim
      
      master_if[i] = new axi_master_if_vc < smpl_cfg > (sc_gen_unique_name("Master-if"));
      master_if[i]->clk(clk);
      master_if[i]->rst_n(rst_n);
      // Pass the address Map
      for (int n=0; n<smpl_cfg::SLAVE_NUM; ++n) // Iterate Slaves
        for (int s=0; s<2; ++s) // Iterate Begin-End Values
          master_if[i]->addr_map[n][s](addr_map[n][s]);
      
      master_if[i]->THIS_ID(NODE_IDS_MASTER[i]);
      
      // Master-AXI-Side
      master_if[i]->ar_in(ar_in[i]);
      master_if[i]->r_out(r_out[i]);
      
      master_if[i]->aw_in(aw_in[i]);
      master_if[i]->w_in(w_in[i]);
      master_if[i]->b_out(b_out[i]);
      // Read-NoC
      master_if[i]->rd_flit_data_out(rreq_m_chan_data[i]);
      master_if[i]->rd_flit_cr_in(rreq_m_chan_cr[i]);
      master_if[i]->rd_flit_data_in(rresp_m_chan_data[i]);
      master_if[i]->rd_flit_cr_out(rresp_m_chan_cr[i]);
      // Write-NoC
      master_if[i]->wr_flit_data_out(wreq_m_chan_data[i]);
      master_if[i]->wr_flit_cr_in(wreq_m_chan_cr[i]);
      master_if[i]->wr_flit_data_in(wresp_m_chan_data[i]);
      master_if[i]->wr_flit_cr_out(wresp_m_chan_cr[i]);
      
      
      // Buffer Connectivity
      // Read Request
      rreq_m_buf_data[i].clk(clk);
      rreq_m_buf_data[i].rst(rst_n);
      rreq_m_buf_data[i].enq(rreq_m_chan_data[i]);
      rreq_m_buf_data[i].deq(chan_inj_rd_data[col][row]);
  
      rreq_m_buf_cr[i].clk(clk);
      rreq_m_buf_cr[i].rst(rst_n);
      rreq_m_buf_cr[i].enq(chan_inj_rd_cr[col][row]);
      rreq_m_buf_cr[i].deq(rreq_m_chan_cr[i]);
  
      // Read Response
      rresp_m_buf_data[i].clk(clk);
      rresp_m_buf_data[i].rst(rst_n);
      rresp_m_buf_data[i].enq( chan_ej_rd_data[col][row]);
      rresp_m_buf_data[i].deq(rresp_m_chan_data[i]);
  
      rresp_m_buf_cr[i].clk(clk);
      rresp_m_buf_cr[i].rst(rst_n);
      rresp_m_buf_cr[i].enq(rresp_m_chan_cr[i]);
      rresp_m_buf_cr[i].deq(chan_ej_rd_cr[col][row]);
  
      // Write Request
      wreq_m_buf_data[i].clk(clk);
      wreq_m_buf_data[i].rst(rst_n);
      wreq_m_buf_data[i].enq(wreq_m_chan_data[i]);
      wreq_m_buf_data[i].deq( chan_inj_wr_data[col][row]);
  
      wreq_m_buf_cr[i].clk(clk);
      wreq_m_buf_cr[i].rst(rst_n);
      wreq_m_buf_cr[i].enq(chan_inj_wr_cr[col][row]);
      wreq_m_buf_cr[i].deq(wreq_m_chan_cr[i]);
  
      // Write Response
      wresp_m_buf_data[i].clk(clk);
      wresp_m_buf_data[i].rst(rst_n);
      wresp_m_buf_data[i].enq(chan_ej_wr_data[col][row]);
      wresp_m_buf_data[i].deq(wresp_m_chan_data[i]);
  
      wresp_m_buf_cr[i].clk(clk);
      wresp_m_buf_cr[i].rst(rst_n);
      wresp_m_buf_cr[i].enq(wresp_m_chan_cr[i]);
      wresp_m_buf_cr[i].deq(chan_ej_wr_cr[col][row]);
      
    }
    // -o-o-o-o-o-o-o-o-o- //
    // -o-o-o-o-o-o-o-o-o- //
    
    for (int row=0; row<DIM_Y; ++row) rtr_id_y_req[row] = row;
    for (int col=0; col<DIM_X; ++col) rtr_id_x_req[col] = col;
    // --- NoC Connectivity --- //
    // Req/Fwd Routers
    for(int row=0; row<DIM_Y; ++row) {
      for (int col=0; col<DIM_X; ++col) {
  
        rtr_inst[col][row].clk(clk);
        rtr_inst[col][row].rst_n(rst_n);
        rtr_inst[col][row].route_lut[0](route_lut[0][0]);
        rtr_inst[col][row].id_x(rtr_id_x_req[col]);
        rtr_inst[col][row].id_y(rtr_id_y_req[row]);
  
        rtr_inst[col][row].data_in[0] (chan_hor_right_data[col][row]);
        rtr_inst[col][row].cr_out[0]  (chan_hor_right_cr[col][row]);
        rtr_inst[col][row].data_out[0](chan_hor_left_data[col][row]);
        rtr_inst[col][row].cr_in[0]   (chan_hor_left_cr[col][row]);
  
        rtr_inst[col][row].data_in[1] (chan_hor_left_data[col+1][row]);
        rtr_inst[col][row].cr_out[1]  (chan_hor_left_cr[col+1][row]);
        rtr_inst[col][row].data_out[1](chan_hor_right_data[col+1][row]);
        rtr_inst[col][row].cr_in[1]   (chan_hor_right_cr[col+1][row]);
  
        rtr_inst[col][row].data_in[2] (chan_ver_up_data[col][row]);
        rtr_inst[col][row].cr_out[2]  (chan_ver_up_cr[col][row]);
        rtr_inst[col][row].data_out[2](chan_ver_down_data[col][row]);
        rtr_inst[col][row].cr_in[2]   (chan_ver_down_cr[col][row]);
  
        rtr_inst[col][row].data_in[3] (chan_ver_down_data[col][row+1]);
        rtr_inst[col][row].cr_out[3]  (chan_ver_down_cr[col][row+1]);
        rtr_inst[col][row].data_out[3](chan_ver_up_data[col][row+1]);
        rtr_inst[col][row].cr_in[3]   (chan_ver_up_cr[col][row+1]);
  
        rtr_inst[col][row].data_in[4] (chan_inj_rd_data[col][row]);
        rtr_inst[col][row].cr_out[4]  (chan_inj_rd_cr[col][row]);
        rtr_inst[col][row].data_out[4](chan_ej_rd_data[col][row]);
        rtr_inst[col][row].cr_in[4]   (chan_ej_rd_cr[col][row]);
  
        rtr_inst[col][row].data_in[5] (chan_inj_wr_data[col][row]);
        rtr_inst[col][row].cr_out[5]  (chan_inj_wr_cr[col][row]);
        rtr_inst[col][row].data_out[5](chan_ej_wr_data[col][row]);
        rtr_inst[col][row].cr_in[5]   (chan_ej_wr_cr[col][row]);
      }
    }
  }; // End of constructor

private:
}; // End of SC_MODULE

#endif // AXI4_TOP_IC_H
