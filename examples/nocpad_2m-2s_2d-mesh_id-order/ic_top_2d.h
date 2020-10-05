#ifndef AXI4_TOP_IC_H
#define AXI4_TOP_IC_H

#pragma once

#include "../../src/axi_master_if.h"
#include "../../src/axi_slave_if.h"

#include "../../src/router_wh.h"

#include "systemc.h"
#include "nvhls_connections.h"

#pragma hls_design top

// Bundle of configuration parameters
template <
  unsigned char MASTER_NUM_ , unsigned char SLAVE_NUM_,
  unsigned char RD_LANES_   , unsigned char WR_LANES_,
  unsigned char RREQ_PHITS_ , unsigned char RRESP_PHITS_,
  unsigned char WREQ_PHITS_ , unsigned char WRESP_PHITS_,
  unsigned char ORD_SCHEME_
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
};

// the used configuration. 2 Masters/Slaves, 64bit AXI, 2.4.4.1 phit flits
typedef cfg<2, 2, 8, 8, 4, 4, 4, 4, 1> smpl_cfg;

SC_MODULE(ic_top) {
public:
  // typedef matchlib's axi with the "standard" configuration
  typedef typename axi::axi4<axi::cfg::standard_duth> axi4_;
  
  // typedef the 4 kind of flits(RD/WR Req/Resp) depending their size
  typedef flit_dnp<smpl_cfg::RREQ_PHITS>  rreq_flit_t;
  typedef flit_dnp<smpl_cfg::RRESP_PHITS> rresp_flit_t;
  typedef flit_dnp<smpl_cfg::WREQ_PHITS>  wreq_flit_t;
  typedef flit_dnp<smpl_cfg::WRESP_PHITS> wresp_flit_t;
    
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
  axi_master_if < smpl_cfg > *master_if[smpl_cfg::MASTER_NUM];
  axi_slave_if  < smpl_cfg > *slave_if[smpl_cfg::SLAVE_NUM];
  
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
  router_wh_top< 4+2, 4+2, rreq_flit_t, 5, DIM_X>   rtr_req[DIM_X][DIM_Y];
  
  Connections::Combinational<wreq_flit_t>    chan_hor_right_req[DIM_X+1][DIM_Y];
  Connections::Combinational<wreq_flit_t>    chan_hor_left_req[DIM_X+1][DIM_Y];
  Connections::Combinational<wreq_flit_t>    chan_ver_up_req[DIM_X][DIM_Y+1];
  Connections::Combinational<wreq_flit_t>    chan_ver_down_req[DIM_X][DIM_Y+1];
  
  Connections::Combinational<wreq_flit_t>    chan_inj_wreq[DIM_X][DIM_Y];
  Connections::Combinational<wreq_flit_t>    chan_inj_rreq[DIM_X][DIM_Y];
  
  Connections::Combinational<wreq_flit_t>    chan_ej_wreq[DIM_X][DIM_Y];
  Connections::Combinational<wreq_flit_t>    chan_ej_rreq[DIM_X][DIM_Y];
  
  
  // RESP Router + In/Out Channels
  router_wh_top< 4+2, 4+2, rresp_flit_t, 5, DIM_X>  *rtr_resp[DIM_X][DIM_Y];
  
  Connections::Combinational<rreq_flit_t>    chan_hor_right_resp[DIM_X+1][DIM_Y];
  Connections::Combinational<rreq_flit_t>    chan_hor_left_resp[DIM_X+1][DIM_Y];
  Connections::Combinational<rreq_flit_t>    chan_ver_up_resp[DIM_X][DIM_Y+1];
  Connections::Combinational<rreq_flit_t>    chan_ver_down_resp[DIM_X][DIM_Y+1];
  
  
  Connections::Combinational<rresp_flit_t>    chan_inj_wresp[DIM_X][DIM_Y];
  Connections::Combinational<rresp_flit_t>    chan_inj_rresp[DIM_X][DIM_Y];
  
  Connections::Combinational<rresp_flit_t>    chan_ej_wresp[DIM_X][DIM_Y];
  Connections::Combinational<rresp_flit_t>    chan_ej_rresp[DIM_X][DIM_Y];
  
  
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
      
      slave_if[j] = new axi_slave_if < smpl_cfg > (sc_gen_unique_name("Slave-if"));
      slave_if[j]->clk(clk);
      slave_if[j]->rst_n(rst_n);
      
      slave_if[j]->THIS_ID(NODE_IDS_SLAVE[j]);
      slave_if[j]->slave_base_addr(addr_map[j][0]);
      // Read-NoC
      slave_if[j]->rd_flit_in(chan_ej_rreq[col][row]);
      slave_if[j]->rd_flit_out(chan_inj_rresp[col][row]);
      // Write-NoC
      slave_if[j]->wr_flit_in(chan_ej_wreq[col][row]);
      slave_if[j]->wr_flit_out(chan_inj_wresp[col][row]);
      // Slave-Side
      slave_if[j]->ar_out(ar_out[j]);
      slave_if[j]->r_in(r_in[j]);
      
      slave_if[j]->aw_out(aw_out[j]);
      slave_if[j]->w_out(w_out[j]);
      slave_if[j]->b_in(b_in[j]);
    }
    
    // ------------------------------ //
    // --- MASTER-IFs Connectivity--- //
    // ------------------------------ //
    for (int i=0; i<smpl_cfg::MASTER_NUM; ++i) {
      NODE_IDS_MASTER[i] = smpl_cfg::SLAVE_NUM + i;
  
      unsigned col = (smpl_cfg::SLAVE_NUM + i) % DIM_X; // aka x dim
      unsigned row = (smpl_cfg::SLAVE_NUM + i) / DIM_X; // aka y dim
      
      master_if[i] = new axi_master_if < smpl_cfg > (sc_gen_unique_name("Master-if"));
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
      master_if[i]->rd_flit_out(chan_inj_rreq[col][row]);
      master_if[i]->rd_flit_in(chan_ej_rresp[col][row]);
      // Write-NoC
      master_if[i]->wr_flit_out(chan_inj_wreq[col][row]);
      master_if[i]->wr_flit_in(chan_ej_wresp[col][row]);
    }
    // -o-o-o-o-o-o-o-o-o- //
    // -o-o-o-o-o-o-o-o-o- //
    
    for (int row=0; row<DIM_Y; ++row) rtr_id_y_req[row] = row;
    for (int col=0; col<DIM_X; ++col) rtr_id_x_req[col] = col;
    // --- NoC Connectivity --- //
    // Req/Fwd Routers
    for(int row=0; row<DIM_Y; ++row) {
      for (int col=0; col<DIM_X; ++col) {
        
        rtr_req[col][row].clk(clk);
        rtr_req[col][row].rst_n(rst_n);
        rtr_req[col][row].route_lut[0](route_lut[0][0]);
        rtr_req[col][row].id_x(rtr_id_x_req[col]);
        rtr_req[col][row].id_y(rtr_id_y_req[row]);
        
        rtr_req[col][row].data_in[0](chan_hor_right_req[col][row]);
        rtr_req[col][row].data_out[0](chan_hor_left_req[col][row]);

        rtr_req[col][row].data_in[1](chan_hor_left_req[col+1][row]);
        rtr_req[col][row].data_out[1](chan_hor_right_req[col+1][row]);
        
        rtr_req[col][row].data_in[2](chan_ver_up_req[col][row]);
        rtr_req[col][row].data_out[2](chan_ver_down_req[col][row]);
        
        rtr_req[col][row].data_in[3](chan_ver_down_req[col][row+1]);
        rtr_req[col][row].data_out[3](chan_ver_up_req[col][row+1]);
  
        rtr_req[col][row].data_in[4](chan_inj_rreq[col][row]);
        rtr_req[col][row].data_out[4](chan_ej_rreq[col][row]);

        rtr_req[col][row].data_in[5](chan_inj_wreq[col][row]);
        rtr_req[col][row].data_out[5](chan_ej_wreq[col][row]);
      }
    }
    
    for (int row=0; row<DIM_Y; ++row) rtr_id_y_resp[row] = (row);
    for (int col=0; col<DIM_X; ++col) rtr_id_x_resp[col] = (col);
    // Resp/Bck Router
    for(int row=0; row<DIM_Y; ++row) {
      for (int col=0; col<DIM_X; ++col) {
        rtr_resp[col][row] = new router_wh_top< 4+2, 4+2, rresp_flit_t, 5, DIM_X> (sc_gen_unique_name("Router-resp"));
        rtr_resp[col][row]->clk(clk);
        rtr_resp[col][row]->rst_n(rst_n);
        rtr_resp[col][row]->route_lut[0](route_lut[0][0]);
        rtr_resp[col][row]->id_x(rtr_id_x_resp[col]);
        rtr_resp[col][row]->id_y(rtr_id_y_resp[row]);
  
        rtr_resp[col][row]->data_in[0](chan_hor_right_resp[col][row]);
        rtr_resp[col][row]->data_out[0](chan_hor_left_resp[col][row]);
  
        rtr_resp[col][row]->data_in[1](chan_hor_left_resp[col+1][row]);
        rtr_resp[col][row]->data_out[1](chan_hor_right_resp[col+1][row]);
  
        rtr_resp[col][row]->data_in[2](chan_ver_up_resp[col][row]);
        rtr_resp[col][row]->data_out[2](chan_ver_down_resp[col][row]);
  
        rtr_resp[col][row]->data_in[3](chan_ver_down_resp[col][row+1]);
        rtr_resp[col][row]->data_out[3](chan_ver_up_resp[col][row+1]);
  
        rtr_resp[col][row]->data_in[4](chan_inj_rresp[col][row]);
        rtr_resp[col][row]->data_out[4](chan_ej_rresp[col][row]);
  
        rtr_resp[col][row]->data_in[5](chan_inj_wresp[col][row]);
        rtr_resp[col][row]->data_out[5](chan_ej_wresp[col][row]);
      }
    }
  }; // End of constructor

private:
}; // End of SC_MODULE

#endif // AXI4_TOP_IC_H
