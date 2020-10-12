#ifndef _ACE_IC_TOP_H_
#define _ACE_IC_TOP_H_

#pragma once

#include "../../src/ace/acelite_master_if_con.h"
#include "../../src/ace/ace_master_if_con.h"
#include "../../src/ace/ace_slave_if_con.h"
#include "../../src/ace/ace_home.h"

#include "../../src/router_wh.h"

#include "systemc.h"
#include "nvhls_connections.h"

#pragma hls_design top

// Bundle of configuration parameters
template <
  unsigned char HOME_NUM_,
  unsigned char FULL_MASTER_NUM_, unsigned char LITE_MASTER_NUM_ , unsigned char SLAVE_NUM_,
  unsigned char RD_LANES_   , unsigned char WR_LANES_,
  unsigned char RREQ_PHITS_ , unsigned char RRESP_PHITS_,
  unsigned char WREQ_PHITS_ , unsigned char WRESP_PHITS_,
  unsigned char CREQ_PHITS_ , unsigned char CRESP_PHITS_
>
struct cfg {
  static const unsigned char HOME_NUM    = HOME_NUM_;
  static const unsigned char FULL_MASTER_NUM  = FULL_MASTER_NUM_;
  static const unsigned char LITE_MASTER_NUM  = LITE_MASTER_NUM_;
  static const unsigned char ALL_MASTER_NUM   = FULL_MASTER_NUM_ + LITE_MASTER_NUM;
  static const unsigned char SLAVE_NUM   = SLAVE_NUM_;
  static const unsigned char RD_LANES    = RD_LANES_;
  static const unsigned char WR_LANES    = WR_LANES_;
  static const unsigned char RREQ_PHITS  = RREQ_PHITS_;
  static const unsigned char RRESP_PHITS = RRESP_PHITS_;
  static const unsigned char WREQ_PHITS  = WREQ_PHITS_;
  static const unsigned char WRESP_PHITS = WRESP_PHITS_;
  static const unsigned char CREQ_PHITS  = CREQ_PHITS_;
  static const unsigned char CRESP_PHITS = CRESP_PHITS_;
};

// the used configuration.1 Home, 2 Full ACE Masters, 2 ACE-Lite Masters, 2 Slaves
typedef cfg<1, 2, 2, 2,
            (ace::ace5<axi::cfg::ace>::C_CACHE_WIDTH >> 3), //bits to bytes
            (ace::ace5<axi::cfg::ace>::C_CACHE_WIDTH >> 3), //bits to bytes
            (ace::ace5<axi::cfg::ace>::C_CACHE_WIDTH < 64) ? 4 : (ace::ace5<axi::cfg::ace>::C_CACHE_WIDTH >> 4), //bits to phits
            (ace::ace5<axi::cfg::ace>::C_CACHE_WIDTH < 64) ? 4 : (ace::ace5<axi::cfg::ace>::C_CACHE_WIDTH >> 4), //bits to phits
            (ace::ace5<axi::cfg::ace>::C_CACHE_WIDTH < 64) ? 4 : (ace::ace5<axi::cfg::ace>::C_CACHE_WIDTH >> 4), //bits to phits
            1,
            3,
            (ace::ace5<axi::cfg::ace>::C_CACHE_WIDTH < 64) ? 4 : (ace::ace5<axi::cfg::ace>::C_CACHE_WIDTH >> 4) //bits to phits
           > smpl_cfg;

SC_MODULE(ic_top) {
public:
  // typedef matchlib's axi with the "standard" configuration
  typedef typename ace::ace5<axi::cfg::ace> ace5_;
  
  // typedef the 4 kind of flits(RD/WR Req/Resp) depending their size
  typedef flit_dnp<smpl_cfg::RREQ_PHITS>  rreq_flit_t;
  typedef flit_dnp<smpl_cfg::RRESP_PHITS> rresp_flit_t;
  typedef flit_dnp<smpl_cfg::WREQ_PHITS>  wreq_flit_t;
  typedef flit_dnp<smpl_cfg::WRESP_PHITS> wresp_flit_t;
  
  typedef flit_dnp<smpl_cfg::CREQ_PHITS>  creq_flit_t;
  typedef flit_dnp<smpl_cfg::CRESP_PHITS> cresp_flit_t;
  
  typedef flit_ack ack_flit_t;
  
  static const unsigned NODES = smpl_cfg::SLAVE_NUM+smpl_cfg::ALL_MASTER_NUM+smpl_cfg::HOME_NUM;
  
  sc_in_clk    clk;
  sc_in <bool> rst_n;
  
  // IC's Address map
  sc_in<sc_uint <32> >  addr_map[smpl_cfg::SLAVE_NUM][2]; // [SLAVE_NUM][0:begin, 1: End]
  
  // The Node IDs are passed to IFs as signals
  sc_signal< sc_uint<dnp::D_W> > NODE_IDS[NODES];
  
  // MASTER Side AXI Channels
    // --- ACE --- //
  Connections::Out<ace5_::AC>           ac_out[smpl_cfg::FULL_MASTER_NUM];
  Connections::In<ace5_::CR>            cr_in[smpl_cfg::FULL_MASTER_NUM];
  Connections::In<ace5_::CD>            cd_in[smpl_cfg::FULL_MASTER_NUM];
    // --- Read --- //
  Connections::In<ace5_::AddrPayload>   ar_in[smpl_cfg::ALL_MASTER_NUM];
  Connections::Out<ace5_::ReadPayload>  r_out[smpl_cfg::ALL_MASTER_NUM];
  Connections::In<ace5_::RACK>          rack_in[smpl_cfg::FULL_MASTER_NUM];
    // --- Write --- //
  Connections::In<ace5_::AddrPayload>   aw_in[smpl_cfg::ALL_MASTER_NUM];
  Connections::In<ace5_::WritePayload>  w_in[smpl_cfg::ALL_MASTER_NUM];
  Connections::Out<ace5_::WRespPayload> b_out[smpl_cfg::ALL_MASTER_NUM];
  Connections::In<ace5_::WACK>          wack_in[smpl_cfg::FULL_MASTER_NUM];
  
  // SLAVE Side AXI Channels
  Connections::Out<ace5_::AddrPayload>  ar_out[smpl_cfg::SLAVE_NUM];
  Connections::In<ace5_::ReadPayload>   r_in[smpl_cfg::SLAVE_NUM];
  
  Connections::Out<ace5_::AddrPayload>  aw_out[smpl_cfg::SLAVE_NUM];
  Connections::Out<ace5_::WritePayload> w_out[smpl_cfg::SLAVE_NUM];
  Connections::In<ace5_::WRespPayload>  b_in[smpl_cfg::SLAVE_NUM];
  
  //--- Internals ---//
  // Master/Slave IFs
  ace_master_if_con     < smpl_cfg > *master_if[smpl_cfg::FULL_MASTER_NUM];
  acelite_master_if_con < smpl_cfg > *master_lite_if[smpl_cfg::LITE_MASTER_NUM];
  ace_slave_if_con      < smpl_cfg > *slave_if[smpl_cfg::SLAVE_NUM];
  ace_home              < smpl_cfg > *home[smpl_cfg::HOME_NUM];
  
  // NoC Channels
  // READ Fwd Req, master+home -> slaves+home
  sc_signal<sc_uint<dnp::D_W> > route_rd_req[NODES];
  router_wh_top< smpl_cfg::ALL_MASTER_NUM+smpl_cfg::HOME_NUM, smpl_cfg::SLAVE_NUM+smpl_cfg::HOME_NUM, rreq_flit_t, 4, 0, NODES>  INIT_S1(rtr_rd_req);
  Connections::Combinational<rreq_flit_t>                                     chan_rd_m2r[smpl_cfg::ALL_MASTER_NUM]; // M-IF_to_Rtr
  Connections::Combinational<rreq_flit_t>                                     chan_rd_r2s[smpl_cfg::SLAVE_NUM];  // Rtr_to_S-IF
  Connections::Combinational<rreq_flit_t>                                     chan_rd_req_r2h[smpl_cfg::HOME_NUM]; // M-IF_to_Rtr
  Connections::Combinational<rreq_flit_t>                                     chan_rd_req_h2r[smpl_cfg::HOME_NUM];  // Rtr_to_S-IF
  
  // READ Bck Resp, slaves+home -> home+masters
  sc_signal<sc_uint<dnp::D_W> > route_rd_resp[NODES];
  router_wh_top<smpl_cfg::SLAVE_NUM+smpl_cfg::HOME_NUM, smpl_cfg::ALL_MASTER_NUM+smpl_cfg::HOME_NUM, rresp_flit_t, 4, 0, NODES>  INIT_S1(rtr_rd_resp);
  Connections::Combinational<rresp_flit_t>                                    chan_rd_s2r[smpl_cfg::SLAVE_NUM];  // S-IF_to_Rtr
  Connections::Combinational<rresp_flit_t>                                    chan_rd_r2m[smpl_cfg::ALL_MASTER_NUM]; // Rtr_to_M-IF
  Connections::Combinational<rresp_flit_t>                                    chan_rd_resp_r2h[smpl_cfg::HOME_NUM]; // M-IF_to_Rtr
  Connections::Combinational<rresp_flit_t>                                    chan_rd_resp_h2r[smpl_cfg::HOME_NUM];  // Rtr_to_S-IF
  
  // WRITE fwd Req, Router+In/Out Channels
  sc_signal<sc_uint<dnp::D_W> > route_wr_req[NODES];
  router_wh_top< smpl_cfg::ALL_MASTER_NUM+smpl_cfg::HOME_NUM, smpl_cfg::SLAVE_NUM+smpl_cfg::HOME_NUM, wreq_flit_t, 4, 0, NODES>   INIT_S1(rtr_wr_req);
  Connections::Combinational<wreq_flit_t>                                     chan_wr_m2r[smpl_cfg::ALL_MASTER_NUM]; // M-IF_to_Rtr
  Connections::Combinational<wreq_flit_t>                                     chan_wr_r2s[smpl_cfg::SLAVE_NUM];  // Rtr_to_S-IF
  Connections::Combinational<wreq_flit_t>                                     chan_wr_req_r2h[smpl_cfg::HOME_NUM]; // M-IF_to_Rtr
  Connections::Combinational<wreq_flit_t>                                     chan_wr_req_h2r[smpl_cfg::HOME_NUM];  // Rtr_to_S-IF
  
  // WRITE fwd Req, Router+In/Out Channels
  sc_signal<sc_uint<dnp::D_W> > route_wr_resp[NODES];
  router_wh_top<smpl_cfg::SLAVE_NUM+smpl_cfg::HOME_NUM, smpl_cfg::ALL_MASTER_NUM+smpl_cfg::HOME_NUM, wresp_flit_t, 4, 0, NODES>  INIT_S1(rtr_wr_resp);
  Connections::Combinational<wresp_flit_t>                                    chan_wr_s2r[smpl_cfg::SLAVE_NUM];  // S-IF_to_Rtr
  Connections::Combinational<wresp_flit_t>                                    chan_wr_r2m[smpl_cfg::ALL_MASTER_NUM]; // Rtr_to_M-IF
  Connections::Combinational<wresp_flit_t>                                    chan_wr_resp_r2h[smpl_cfg::HOME_NUM]; // M-IF_to_Rtr
  Connections::Combinational<wresp_flit_t>                                    chan_wr_resp_h2r[smpl_cfg::HOME_NUM];  // Rtr_to_S-IF
  
  // CACHE fwd Req, Router+In/Out Channels
  sc_signal<sc_uint<dnp::D_W> > route_cache_req[NODES];
  router_wh_top<smpl_cfg::HOME_NUM, smpl_cfg::FULL_MASTER_NUM, creq_flit_t, 4, 0, NODES>   INIT_S1(rtr_cache_req);
  Connections::Combinational<creq_flit_t>                                    chan_creq_h2r[smpl_cfg::HOME_NUM];  // Home_to_Rtr
  Connections::Combinational<creq_flit_t>                                    chan_creq_r2m[smpl_cfg::FULL_MASTER_NUM]; // Rtr_to_M-IF
  
  // CACHE Bck Resp, Router+In/Out Channels
  sc_signal<sc_uint<dnp::D_W> > route_cache_resp[NODES];
  router_wh_top<smpl_cfg::FULL_MASTER_NUM, smpl_cfg::HOME_NUM, cresp_flit_t, 4, 0, NODES>   INIT_S1(rtr_cache_resp);
  Connections::Combinational<cresp_flit_t>                                    chan_cresp_m2r[smpl_cfg::FULL_MASTER_NUM];  // M-IF_to_Rtr
  Connections::Combinational<cresp_flit_t>                                    chan_cresp_r2h[smpl_cfg::HOME_NUM]; // Rtr_to_Home
  
  // Master read+write ACKs back to HOME
  sc_signal<sc_uint<dnp::D_W> > route_acks[NODES];
  router_wh_top<smpl_cfg::FULL_MASTER_NUM*2, smpl_cfg::HOME_NUM, ack_flit_t, 4, 0, NODES>   INIT_S1(rtr_acks);
  Connections::Combinational<ack_flit_t>                                    chan_acks_m2r[smpl_cfg::FULL_MASTER_NUM*2];  // M-IF_to_Rtr
  Connections::Combinational<ack_flit_t>                                    chan_acks_r2h[smpl_cfg::HOME_NUM]; // Rtr_to_Home
  
  
  sc_signal< sc_uint<dnp::D_W> > rtr_id_dummmy;
  
  SC_CTOR(ic_top) {
      rtr_id_dummmy = 0;
    
    for (unsigned i=0; i<smpl_cfg::HOME_NUM+smpl_cfg::ALL_MASTER_NUM+smpl_cfg::SLAVE_NUM; ++i)
      NODE_IDS[i] = i;
      
    // ------------------ //
    // --- SLAVE-IFs --- //
    // -------------------//
    for(unsigned char j=0; j<smpl_cfg::SLAVE_NUM; ++j){
      slave_if[j] = new ace_slave_if_con < smpl_cfg > (sc_gen_unique_name("Slave-if"));
      slave_if[j]->clk(clk);
      slave_if[j]->rst_n(rst_n);
      
      slave_if[j]->THIS_ID(NODE_IDS[j]);
      slave_if[j]->slave_base_addr(addr_map[j][0]);
      // Read-NoC
      slave_if[j]->rd_flit_in(chan_rd_r2s[j]);
      slave_if[j]->rd_flit_out(chan_rd_s2r[j]);
      // Write-NoC
      slave_if[j]->wr_flit_in(chan_wr_r2s[j]);
      slave_if[j]->wr_flit_out(chan_wr_s2r[j]);
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
    // Connect each Master-IF to the appropriate channels
    for(int i=0; i<smpl_cfg::FULL_MASTER_NUM; ++i){
      master_if[i] = new ace_master_if_con < smpl_cfg > (sc_gen_unique_name("Master-if"));
      master_if[i]->clk(clk);
      master_if[i]->rst_n(rst_n);
      
      // Pass the address Map
      for (int n=0; n<smpl_cfg::SLAVE_NUM; ++n) // Iterate Slaves
        for (int s=0; s<2; ++s) // Iterate Begin-End Values
          master_if[i]->addr_map[n][s](addr_map[n][s]);
      
      master_if[i]->THIS_ID(NODE_IDS[i+smpl_cfg::SLAVE_NUM]);
      
      // Master-AXI-Side
      master_if[i]->ac_out(ac_out[i]);
      master_if[i]->cr_in(cr_in[i]);
      master_if[i]->cd_in(cd_in[i]);
      
      master_if[i]->ar_in(ar_in[i]);
      master_if[i]->r_out(r_out[i]);
      master_if[i]->rack_in(rack_in[i]);
      
      master_if[i]->aw_in(aw_in[i]);
      master_if[i]->w_in(w_in[i]);
      master_if[i]->b_out(b_out[i]);
      master_if[i]->wack_in(wack_in[i]);
      
      // Read-NoC
      master_if[i]->rd_flit_out(chan_rd_m2r[i]);
      master_if[i]->rd_flit_in(chan_rd_r2m[i]);
      master_if[i]->rack_flit_out(chan_acks_m2r[i]);
      // Write-NoC
      master_if[i]->wr_flit_out(chan_wr_m2r[i]);
      master_if[i]->wr_flit_in(chan_wr_r2m[i]);
      master_if[i]->wack_flit_out(chan_acks_m2r[smpl_cfg::FULL_MASTER_NUM+i]);
      // Cache-NoC
      master_if[i]->cache_flit_in(chan_creq_r2m[i]);
      master_if[i]->cache_flit_out(chan_cresp_m2r[i]);
    }
      
      // Connect ACE LITE Master-IFs to the appropriate channels
      for(int i=0; i<smpl_cfg::LITE_MASTER_NUM; ++i){
        master_lite_if[i] = new acelite_master_if_con < smpl_cfg > (sc_gen_unique_name("Master-Lite-if"));
        master_lite_if[i]->clk(clk);
        master_lite_if[i]->rst_n(rst_n);
        
        // Pass the address Map
        for (int n=0; n<smpl_cfg::SLAVE_NUM; ++n) // Iterate Slaves
          for (int s=0; s<2; ++s) // Iterate Begin-End Values
            master_lite_if[i]->addr_map[n][s](addr_map[n][s]);
  
        master_lite_if[i]->THIS_ID(NODE_IDS[i+smpl_cfg::SLAVE_NUM+smpl_cfg::FULL_MASTER_NUM]);
        
        // Master-AXI-Side
        master_lite_if[i]->ar_in(ar_in[i+smpl_cfg::FULL_MASTER_NUM]);
        master_lite_if[i]->r_out(r_out[i+smpl_cfg::FULL_MASTER_NUM]);
        
        master_lite_if[i]->aw_in(aw_in[i+smpl_cfg::FULL_MASTER_NUM]);
        master_lite_if[i]->w_in(w_in[i+smpl_cfg::FULL_MASTER_NUM]);
        master_lite_if[i]->b_out(b_out[i+smpl_cfg::FULL_MASTER_NUM]);
        
        // Read-NoC
        master_lite_if[i]->rd_flit_out(chan_rd_m2r[i+smpl_cfg::FULL_MASTER_NUM]);
        master_lite_if[i]->rd_flit_in(chan_rd_r2m[i+smpl_cfg::FULL_MASTER_NUM]);
        // Write-NoC
        master_lite_if[i]->wr_flit_out(chan_wr_m2r[i+smpl_cfg::FULL_MASTER_NUM]);
        master_lite_if[i]->wr_flit_in(chan_wr_r2m[i+smpl_cfg::FULL_MASTER_NUM]);
      }
    
    // -------------------- //
    // --- HOME-NODE(s) --- //
    // ---------------------//
    for (unsigned i=0; i<smpl_cfg::HOME_NUM; ++i) {
      home[i] = new ace_home < smpl_cfg > (sc_gen_unique_name("Home-Node"));
      home[i]->clk(clk);
      home[i]->rst_n(rst_n);
  
      // Pass the address Map
      for (int n=0; n<smpl_cfg::SLAVE_NUM; ++n) // Iterate Slaves
        for (int s=0; s<2; ++s) // Iterate Begin-End Values
          home[i]->addr_map[n][s](addr_map[n][s]);
      
      home[i]->THIS_ID(NODE_IDS[i+smpl_cfg::SLAVE_NUM+smpl_cfg::ALL_MASTER_NUM]);
      
      home[i]->cache_req(chan_creq_h2r[i]);
      home[i]->cache_resp(chan_cresp_r2h[i]);
      //home[i]->cache_ack();
      
      home[i]->rd_from_master(chan_rd_req_r2h[i]);
      home[i]->rd_to_master(chan_rd_resp_h2r[i]);
      
      home[i]->rd_to_slave(chan_rd_req_h2r[i]);
      home[i]->rd_from_slave(chan_rd_resp_r2h[i]);
  
      home[i]->wr_from_master(chan_wr_req_r2h[i]);
      home[i]->wr_to_master(chan_wr_resp_h2r[i]);
  
      home[i]->wr_to_slave(chan_wr_req_h2r[i]);
      home[i]->wr_from_slave(chan_wr_resp_r2h[i]);
      
      home[i]->ack_from_master(chan_acks_r2h[i]);
    }
    
    // -o-o-o-o-o-o-o-o-o- //
    // -o-o-o-o-o-o-o-o-o- //
    
    
    // --- NoC Connectivity --- //
    // Read Req/Fwd Router
    for(int i=0; i<smpl_cfg::SLAVE_NUM; ++i)
      route_rd_req[i] = i; // Slaves
    for(int i=0; i<smpl_cfg::ALL_MASTER_NUM; ++i)
      route_rd_req[i+smpl_cfg::SLAVE_NUM] = 0; // Masters
    for(int i=0; i<smpl_cfg::HOME_NUM; ++i)
      route_rd_req[i+smpl_cfg::SLAVE_NUM+smpl_cfg::ALL_MASTER_NUM] = i+smpl_cfg::SLAVE_NUM; // Homes
    
    rtr_rd_req.clk(clk);
    rtr_rd_req.rst_n(rst_n);
    rtr_rd_req.id_x(rtr_id_dummmy);
    rtr_rd_req.id_y(rtr_id_dummmy);
    for (unsigned i=0; i<NODES; ++i)
      rtr_rd_req.route_lut[i](route_rd_req[i]);
    // In from Masters
    for(int i=0; i<smpl_cfg::ALL_MASTER_NUM; ++i)  rtr_rd_req.data_in[i](chan_rd_m2r[i]);
    for(int i=0; i<smpl_cfg::HOME_NUM; ++i)        rtr_rd_req.data_in[smpl_cfg::ALL_MASTER_NUM+i](chan_rd_req_h2r[i]);
    // Out to Slave
    for(int i=0; i<smpl_cfg::SLAVE_NUM; ++i)  rtr_rd_req.data_out[i](chan_rd_r2s[i]);
    for(int i=0; i<smpl_cfg::HOME_NUM; ++i)   rtr_rd_req.data_out[smpl_cfg::SLAVE_NUM+i](chan_rd_req_r2h[i]);
    
    // Read Resp/Bck Router
    for(int i=0; i<smpl_cfg::SLAVE_NUM; ++i)
      route_rd_resp[i] = 0; // Slaves
    for(int i=0; i<smpl_cfg::ALL_MASTER_NUM; ++i)
      route_rd_resp[i+smpl_cfg::SLAVE_NUM] = i; // Masters
    for(int i=0; i<smpl_cfg::HOME_NUM; ++i)
      route_rd_resp[i+smpl_cfg::SLAVE_NUM+smpl_cfg::ALL_MASTER_NUM] = i+smpl_cfg::ALL_MASTER_NUM; // Homes
    
    rtr_rd_resp.clk(clk);
    rtr_rd_resp.rst_n(rst_n);
    rtr_rd_resp.id_x(rtr_id_dummmy);
    rtr_rd_resp.id_y(rtr_id_dummmy);
    for (unsigned i=0; i<NODES; ++i)
      rtr_rd_resp.route_lut[i](route_rd_resp[i]);
    // In from Slaves
    for(int i=0; i<smpl_cfg::SLAVE_NUM; ++i)  rtr_rd_resp.data_in[i](chan_rd_s2r[i]);
    for(int i=0; i<smpl_cfg::HOME_NUM; ++i)   rtr_rd_resp.data_in[smpl_cfg::SLAVE_NUM+i](chan_rd_resp_h2r[i]);
    // Out to Master
    for(int i=0; i<smpl_cfg::ALL_MASTER_NUM; ++i) rtr_rd_resp.data_out[i](chan_rd_r2m[i]);
    for(int i=0; i<smpl_cfg::HOME_NUM; ++i)       rtr_rd_resp.data_out[smpl_cfg::ALL_MASTER_NUM+i](chan_rd_resp_r2h[i]);
    
    // Write Req/Fwd Router
    for(int i=0; i<smpl_cfg::SLAVE_NUM; ++i)
      route_wr_req[i] = i; // Slaves
    for(int i=0; i<smpl_cfg::ALL_MASTER_NUM; ++i)
      route_wr_req[i+smpl_cfg::SLAVE_NUM] = 0; // Masters
    for(int i=0; i<smpl_cfg::HOME_NUM; ++i)
      route_wr_req[i+smpl_cfg::SLAVE_NUM+smpl_cfg::ALL_MASTER_NUM] = i+smpl_cfg::SLAVE_NUM; // Homes
    
    rtr_wr_req.clk(clk);
    rtr_wr_req.rst_n(rst_n);
    rtr_wr_req.id_x(rtr_id_dummmy);
    rtr_wr_req.id_y(rtr_id_dummmy);
    for (unsigned i=0; i<NODES; ++i)
      rtr_wr_req.route_lut[i](route_wr_req[i]);
    // In from Masters
    for(int i=0; i<smpl_cfg::ALL_MASTER_NUM; ++i)  rtr_wr_req.data_in[i](chan_wr_m2r[i]);
    for(int i=0; i<smpl_cfg::HOME_NUM; ++i)        rtr_wr_req.data_in[smpl_cfg::ALL_MASTER_NUM+i](chan_wr_req_h2r[i]);
    // Out to Slaves
    for(int i=0; i<smpl_cfg::SLAVE_NUM; ++i)  rtr_wr_req.data_out[i](chan_wr_r2s[i]);
    for(int i=0; i<smpl_cfg::HOME_NUM; ++i)   rtr_wr_req.data_out[smpl_cfg::SLAVE_NUM+i](chan_wr_req_r2h[i]);
    
    // Write Resp/Bck Router
    for(int i=0; i<smpl_cfg::SLAVE_NUM; ++i)
      route_wr_resp[i] = 0; // Slaves
    for(int i=0; i<smpl_cfg::ALL_MASTER_NUM; ++i)
      route_wr_resp[i+smpl_cfg::SLAVE_NUM] = i; // Masters
    for(int i=0; i<smpl_cfg::HOME_NUM; ++i)
      route_wr_resp[i+smpl_cfg::SLAVE_NUM+smpl_cfg::ALL_MASTER_NUM] = i+smpl_cfg::ALL_MASTER_NUM; // Homes
    
    rtr_wr_resp.clk(clk);
    rtr_wr_resp.rst_n(rst_n);
    rtr_wr_resp.id_x(rtr_id_dummmy);
    rtr_wr_resp.id_y(rtr_id_dummmy);
    for (unsigned i=0; i<NODES; ++i)
      rtr_wr_resp.route_lut[i](route_wr_resp[i]);
    // In from Slaves
    for(int i=0; i<smpl_cfg::SLAVE_NUM; ++i)  rtr_wr_resp.data_in[i](chan_wr_s2r[i]);
    for(int i=0; i<smpl_cfg::HOME_NUM; ++i)   rtr_wr_resp.data_in[smpl_cfg::SLAVE_NUM+i](chan_wr_resp_h2r[i]);
    // Out to Master
    for(int i=0; i<smpl_cfg::ALL_MASTER_NUM; ++i)  rtr_wr_resp.data_out[i](chan_wr_r2m[i]);
    for(int i=0; i<smpl_cfg::HOME_NUM; ++i)        rtr_wr_resp.data_out[smpl_cfg::ALL_MASTER_NUM+i](chan_wr_resp_r2h[i]);
    
    // Cache Req/Fwd Router
    for(int i=0; i<smpl_cfg::SLAVE_NUM; ++i)
      route_cache_req[i] = 0; // Slaves
    for(int i=0; i<smpl_cfg::ALL_MASTER_NUM; ++i)
      route_cache_req[i+smpl_cfg::SLAVE_NUM] = i; // Masters
    for(int i=0; i<smpl_cfg::HOME_NUM; ++i)
      route_cache_req[i+smpl_cfg::SLAVE_NUM+smpl_cfg::ALL_MASTER_NUM] = 0; // Homes
    
    rtr_cache_req.clk(clk);
    rtr_cache_req.rst_n(rst_n);
    rtr_cache_req.id_x(rtr_id_dummmy);
    rtr_cache_req.id_y(rtr_id_dummmy);
    for (unsigned i=0; i<NODES; ++i)
      rtr_cache_req.route_lut[i](route_cache_req[i]);
    // In from Home
    for(int i=0; i<smpl_cfg::HOME_NUM; ++i) {
      rtr_cache_req.data_in[i](chan_creq_h2r[i]);
    }
    // Out to Master
    for(int i=0; i<smpl_cfg::FULL_MASTER_NUM; ++i) {
      rtr_cache_req.data_out[i](chan_creq_r2m[i]);
    }
    
    // Cache Resp/Bck Router
    for(int i=0; i<smpl_cfg::SLAVE_NUM; ++i)
      route_cache_resp[i] = 0; // Slaves
    for(int i=0; i<smpl_cfg::ALL_MASTER_NUM; ++i)
      route_cache_resp[i+smpl_cfg::SLAVE_NUM] = 0; // Masters
    for(int i=0; i<smpl_cfg::HOME_NUM; ++i)
      route_cache_resp[i+smpl_cfg::SLAVE_NUM+smpl_cfg::ALL_MASTER_NUM] = i; // Homes
    
    rtr_cache_resp.clk(clk);
    rtr_cache_resp.rst_n(rst_n);
    rtr_cache_resp.id_x(rtr_id_dummmy);
    rtr_cache_resp.id_y(rtr_id_dummmy);
    for (unsigned i=0; i<NODES; ++i)
      rtr_cache_resp.route_lut[i](route_cache_resp[i]);
    // In from Home
    for(int i=0; i<smpl_cfg::FULL_MASTER_NUM; ++i) {
      rtr_cache_resp.data_in[i](chan_cresp_m2r[i]);
    }
    // Out to Master
    for(int i=0; i<smpl_cfg::HOME_NUM; ++i) {
      rtr_cache_resp.data_out[i](chan_cresp_r2h[i]);
    }
    
    
    // ACKS Router
    for(int i=0; i<smpl_cfg::SLAVE_NUM; ++i)
      route_acks[i] = 0; // Slaves
    for(int i=0; i<smpl_cfg::ALL_MASTER_NUM; ++i)
      route_acks[i+smpl_cfg::SLAVE_NUM] = 0; // Masters
    for(int i=0; i<smpl_cfg::HOME_NUM; ++i)
      route_acks[i+smpl_cfg::SLAVE_NUM+smpl_cfg::ALL_MASTER_NUM] = i; // Homes
      
    rtr_acks.clk(clk);
    rtr_acks.rst_n(rst_n);
    rtr_acks.id_x(rtr_id_dummmy);
    rtr_acks.id_y(rtr_id_dummmy);
    for (unsigned i=0; i<NODES; ++i)
      rtr_acks.route_lut[i](route_acks[i]);
    // In from Home
    for(int i=0; i<(smpl_cfg::FULL_MASTER_NUM*2); ++i) {
      rtr_acks.data_in[i](chan_acks_m2r[i]);
    }
    // Out to HOME
    for(int i=0; i<smpl_cfg::HOME_NUM; ++i) {
      rtr_acks.data_out[i](chan_acks_r2h[i]);
    }
    
  }; // End of constructor

private:
}; // End of SC_MODULE

#endif // _ACE_IC_TOP_H_
