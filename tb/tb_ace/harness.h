#ifndef ACE_IC_HARNESS_H
#define ACE_IC_HARNESS_H

#include "systemc.h"

#include <mc_scverify.h>

#include "./ic_top.h"

#define NVHLS_VERIFY_BLOCKS (ic_top)
#include "nvhls_verify.h"

#include "stdlib.h"
#include <string>

#include "../../tb/tb_ace/acelite_master.h"
#include "../../tb/tb_ace/ace_master.h"
#include "../../tb/tb_ace/ace_slave.h"
#include "../../tb/tb_ace/ace_coherency_checker.h"

#include <iostream>
#include <fstream>

SC_MODULE(harness) {
  typedef typename ace::ace5<axi::cfg::ace> ace5_;
  
  const int CLK_PERIOD = 5;
  const int GEN_CYCLES = 2 * 1000;
  
  const int AXI_GEN_RATE_RD[smpl_cfg::ALL_MASTER_NUM]    = {20, 20, 20, 20};
  const int AXI_GEN_RATE_WR[smpl_cfg::ALL_MASTER_NUM]    = {20, 20, 20, 20};
  const int ACE_GEN_RATE_CACHE[smpl_cfg::ALL_MASTER_NUM] = {10, 10, 10, 10};
  
  const int AXI_STALL_RATE_RD = 00;
  const int AXI_STALL_RATE_WR = 00;
  
  const int DRAIN_CYCLES = GEN_CYCLES/10;
   
  sc_clock        clk; //clock signal
  sc_signal<bool> rst_n;
  
  sc_signal<bool> stop_gen;
  
  // --- Scoreboards --- //
  // Scoreboards refer to the receiver of the queue.
  // I.e. the receiver checks what is expected to be received. Thus sender must take care to push Transactions to the appropriate queue
  sc_mutex                                                      sb_lock;
  std::vector< std::deque< msg_tb_wrap<ace5_::AddrPayload> > >  sb_rd_req_q;
  std::vector< std::deque< msg_tb_wrap<ace5_::ReadPayload> > >  sb_rd_resp_q;
  
  std::vector< std::deque< msg_tb_wrap<ace5_::AddrPayload > > >  sb_wr_req_q;
  std::vector< std::deque< msg_tb_wrap<ace5_::WritePayload> > >  sb_wr_data_q;
  std::vector< std::deque< msg_tb_wrap<ace5_::WRespPayload> > >  sb_wr_resp_q;
  
  std::vector< std::deque< msg_tb_wrap<ace5_::AddrPayload> > >   sb_coherent_access_q;
  std::vector< std::deque< msg_tb_wrap<ace5_::AC> > >            sb_snoop_req_q;
  std::vector< std::deque< msg_tb_wrap<ace5_::CR> > >            sb_snoop_resp_q;
  std::vector< std::deque< msg_tb_wrap<ace5_::CD> > >            sb_snoop_data_resp_q;
  
  CCS_DESIGN(ic_top) interconnect;
  // ic_top interconnect("interconnect");
  
  ace_master<smpl_cfg::RD_LANES, smpl_cfg::RD_LANES, smpl_cfg::WR_LANES, smpl_cfg::WR_LANES, smpl_cfg::ALL_MASTER_NUM, smpl_cfg::SLAVE_NUM> *master[smpl_cfg::FULL_MASTER_NUM];
  acelite_master<smpl_cfg::RD_LANES, smpl_cfg::RD_LANES, smpl_cfg::WR_LANES, smpl_cfg::WR_LANES, smpl_cfg::ALL_MASTER_NUM, smpl_cfg::SLAVE_NUM> *master_lite[smpl_cfg::LITE_MASTER_NUM];
  ace_slave<smpl_cfg::RD_LANES, smpl_cfg::RD_LANES, smpl_cfg::WR_LANES, smpl_cfg::WR_LANES, smpl_cfg::ALL_MASTER_NUM, smpl_cfg::SLAVE_NUM>  *slave[smpl_cfg::SLAVE_NUM];
  
  ace_coherency_checker<smpl_cfg::RD_LANES, smpl_cfg::RD_LANES, smpl_cfg::WR_LANES, smpl_cfg::WR_LANES, smpl_cfg::FULL_MASTER_NUM, smpl_cfg::LITE_MASTER_NUM, smpl_cfg::SLAVE_NUM>  coherency_checker;
  
  sc_signal< sc_uint<32> >  addr_map[smpl_cfg::SLAVE_NUM][2];
  
  // Channels connecting Master <--> IC
  Connections::Combinational<ace5_::AC>           master_snoop[smpl_cfg::FULL_MASTER_NUM];
  Connections::Combinational<ace5_::CR>           master_snoop_resp[smpl_cfg::FULL_MASTER_NUM];
  Connections::Combinational<ace5_::CD>           master_snoop_data[smpl_cfg::FULL_MASTER_NUM];
  
  Connections::Combinational<ace5_::AddrPayload>  master_rd_req[smpl_cfg::ALL_MASTER_NUM];
  Connections::Combinational<ace5_::ReadPayload>  master_rd_resp[smpl_cfg::ALL_MASTER_NUM];
  Connections::Combinational<ace5_::RACK>         master_rd_ack[smpl_cfg::FULL_MASTER_NUM];
  
  Connections::Combinational<ace5_::AddrPayload>   master_wr_req[smpl_cfg::ALL_MASTER_NUM];
  Connections::Combinational<ace5_::WritePayload>  master_wr_data[smpl_cfg::ALL_MASTER_NUM];
  Connections::Combinational<ace5_::WRespPayload>  master_wr_resp[smpl_cfg::ALL_MASTER_NUM];
  Connections::Combinational<ace5_::WACK>          master_wr_ack[smpl_cfg::FULL_MASTER_NUM];
  
  // Channels connecting IC <--> Slave
  Connections::Combinational<ace5_::AddrPayload>  slave_rd_req[smpl_cfg::SLAVE_NUM];
  Connections::Combinational<ace5_::ReadPayload>  slave_rd_resp[smpl_cfg::SLAVE_NUM];
  
  Connections::Combinational<ace5_::AddrPayload>   slave_wr_req[smpl_cfg::SLAVE_NUM];
  Connections::Combinational<ace5_::WritePayload>  slave_wr_data[smpl_cfg::SLAVE_NUM];
  Connections::Combinational<ace5_::WRespPayload>  slave_wr_resp[smpl_cfg::SLAVE_NUM];
  
  
  SC_CTOR(harness) :
    clk("clock",10,SC_NS,0.5,0.0,SC_NS),
    rst_n("rst_n"),
    stop_gen("stop_gen"),
    
    sb_lock(),
    sb_rd_req_q(smpl_cfg::SLAVE_NUM),
    sb_rd_resp_q(smpl_cfg::ALL_MASTER_NUM),
    
    sb_wr_req_q(smpl_cfg::SLAVE_NUM),
    sb_wr_data_q(smpl_cfg::SLAVE_NUM),
    sb_wr_resp_q(smpl_cfg::ALL_MASTER_NUM),

    sb_coherent_access_q(smpl_cfg::ALL_MASTER_NUM),
    sb_snoop_req_q(smpl_cfg::FULL_MASTER_NUM),
    sb_snoop_resp_q(smpl_cfg::FULL_MASTER_NUM),
    sb_snoop_data_resp_q(smpl_cfg::FULL_MASTER_NUM),
    
    coherency_checker("coherency_checker"),
    interconnect("interconnect")
  {
    // Construct Components
    for (int i=0; i<smpl_cfg::FULL_MASTER_NUM; ++i)
      master[i] = new ace_master<smpl_cfg::RD_LANES, smpl_cfg::RD_LANES, smpl_cfg::WR_LANES, smpl_cfg::WR_LANES, smpl_cfg::ALL_MASTER_NUM, smpl_cfg::SLAVE_NUM>(sc_gen_unique_name("master"));
    for (int i=0; i<smpl_cfg::LITE_MASTER_NUM; ++i)
      master_lite[i] = new acelite_master<smpl_cfg::RD_LANES, smpl_cfg::RD_LANES, smpl_cfg::WR_LANES, smpl_cfg::WR_LANES, smpl_cfg::ALL_MASTER_NUM, smpl_cfg::SLAVE_NUM>(sc_gen_unique_name("master-lite"));
    for (int i=0; i<smpl_cfg::SLAVE_NUM; ++i)
      slave[i]  = new ace_slave <smpl_cfg::RD_LANES, smpl_cfg::RD_LANES, smpl_cfg::WR_LANES, smpl_cfg::WR_LANES, smpl_cfg::ALL_MASTER_NUM, smpl_cfg::SLAVE_NUM>(sc_gen_unique_name("slave"));
    
    std::cout << "---  Binding...  ---\n";
    std::cout.flush();
  
    unsigned fu = smpl_cfg::SLAVE_NUM;
    addr_map[0][0] = 0;
    addr_map[0][1] = 0x0ffff;
    addr_map[1][0] = 0x10000;
    addr_map[1][1] = 0x2ffff;
    
    // BINDING - START
    
    // COHERENCY CHECKER
    coherency_checker.sb_lock      = &sb_lock;      // Scoreboard by Ref
    coherency_checker.sb_rd_req_q  = &sb_rd_req_q;  // Scoreboard by Ref
    coherency_checker.sb_rd_resp_q = &sb_rd_resp_q; // Scoreboard by Ref
  
    coherency_checker.sb_wr_req_q  = &sb_wr_req_q;  // Scoreboard by Ref
    coherency_checker.sb_wr_data_q = &sb_wr_data_q; // Scoreboard by Ref
    coherency_checker.sb_wr_resp_q = &sb_wr_resp_q; // Scoreboard by Ref
  
    coherency_checker.sb_coherent_access_q  = &sb_coherent_access_q;  // Scoreboard by Ref
    coherency_checker.sb_snoop_req_q        = &sb_snoop_req_q;  // Scoreboard by Ref
    coherency_checker.sb_snoop_resp_q       = &sb_snoop_resp_q;  // Scoreboard by Ref
    coherency_checker.sb_snoop_data_resp_q  = &sb_snoop_data_resp_q;  // Scoreboard by Ref
  
    coherency_checker.stop_gen(stop_gen);
  
    coherency_checker.clk(clk);
    coherency_checker.rst_n(rst_n);
  
    for(int j=0; j<smpl_cfg::SLAVE_NUM; ++j) {
      coherency_checker.addr_map[j][0](addr_map[j][0]);
      coherency_checker.addr_map[j][1](addr_map[j][1]);
    }
    
    // IC-Clk/Rst
    interconnect.clk(clk);
    interconnect.rst_n(rst_n);
    
    // FULL ACE MASTER
    for (int i=0; i<smpl_cfg::FULL_MASTER_NUM; ++i) {
      master[i]->sb_lock      = &sb_lock;      // Scoreboard by Ref
      master[i]->sb_rd_req_q  = &sb_rd_req_q;  // Scoreboard by Ref
      master[i]->sb_rd_resp_q = &sb_rd_resp_q; // Scoreboard by Ref
      
      master[i]->sb_wr_req_q  = &sb_wr_req_q;  // Scoreboard by Ref
      master[i]->sb_wr_data_q = &sb_wr_data_q; // Scoreboard by Ref
      master[i]->sb_wr_resp_q = &sb_wr_resp_q; // Scoreboard by Ref
      
      master[i]->sb_coherent_access_q  = &sb_coherent_access_q;  // Scoreboard by Ref
      master[i]->sb_snoop_req_q        = &sb_snoop_req_q;  // Scoreboard by Ref
      master[i]->sb_snoop_resp_q       = &sb_snoop_resp_q;  // Scoreboard by Ref
      master[i]->sb_snoop_data_resp_q  = &sb_snoop_data_resp_q;  // Scoreboard by Ref
      
      master[i]->MASTER_ID    = i+smpl_cfg::SLAVE_NUM;
      master[i]->AXI_GEN_RATE_RD     = AXI_GEN_RATE_RD[i];
      master[i]->AXI_GEN_RATE_WR     = AXI_GEN_RATE_WR[i];
      master[i]->ACE_GEN_RATE_CACHE  = ACE_GEN_RATE_CACHE[i];
      master[i]->stop_gen(stop_gen);
      
      master[i]->clk(clk);
      master[i]->rst_n(rst_n);
      
      for(int j=0; j<smpl_cfg::SLAVE_NUM; ++j) {
        master[i]->addr_map[j][0](addr_map[j][0]);
        master[i]->addr_map[j][1](addr_map[j][1]);
      }
      
      master[i]->ac_in(master_snoop[i]);
      master[i]->cr_out(master_snoop_resp[i]);
      master[i]->cd_out(master_snoop_data[i]);
      
      
      master[i]->ar_out(master_rd_req[i]);
      master[i]->r_in(master_rd_resp[i]);
      master[i]->rack_out(master_rd_ack[i]);
      
      master[i]->aw_out(master_wr_req[i]);
      master[i]->w_out(master_wr_data[i]);
      master[i]->b_in(master_wr_resp[i]);
      master[i]->wack_out(master_wr_ack[i]);
      
      // Connect masters to IC
      interconnect.ac_out[i](master_snoop[i]);
      interconnect.cr_in[i](master_snoop_resp[i]);
      interconnect.cd_in[i](master_snoop_data[i]);
            
      interconnect.ar_in[i](master_rd_req[i]);
      interconnect.r_out[i](master_rd_resp[i]);
      interconnect.rack_in[i](master_rd_ack[i]);
      
      interconnect.aw_in[i](master_wr_req[i]);
      interconnect.w_in[i](master_wr_data[i]);
      interconnect.b_out[i](master_wr_resp[i]);
      interconnect.wack_in[i](master_wr_ack[i]);
    }
  
    // ACE-LITE MASTER
    for (int i=0; i<smpl_cfg::LITE_MASTER_NUM; ++i) {
      master_lite[i]->sb_lock      = &sb_lock;      // Scoreboard by Ref
      master_lite[i]->sb_rd_req_q  = &sb_rd_req_q;  // Scoreboard by Ref
      master_lite[i]->sb_rd_resp_q = &sb_rd_resp_q; // Scoreboard by Ref
    
      master_lite[i]->sb_wr_req_q  = &sb_wr_req_q;  // Scoreboard by Ref
      master_lite[i]->sb_wr_data_q = &sb_wr_data_q; // Scoreboard by Ref
      master_lite[i]->sb_wr_resp_q = &sb_wr_resp_q; // Scoreboard by Ref
    
      master_lite[i]->sb_coherent_access_q  = &sb_coherent_access_q;  // Scoreboard by Ref
      master_lite[i]->sb_snoop_req_q        = &sb_snoop_req_q;  // Scoreboard by Ref
      master_lite[i]->sb_snoop_resp_q       = &sb_snoop_resp_q;  // Scoreboard by Ref
      master_lite[i]->sb_snoop_data_resp_q  = &sb_snoop_data_resp_q;  // Scoreboard by Ref
    
      master_lite[i]->MASTER_ID    = i+smpl_cfg::SLAVE_NUM+smpl_cfg::FULL_MASTER_NUM;
      master_lite[i]->AXI_GEN_RATE_RD     = AXI_GEN_RATE_RD[i+smpl_cfg::FULL_MASTER_NUM];
      master_lite[i]->AXI_GEN_RATE_WR     = AXI_GEN_RATE_WR[i+smpl_cfg::FULL_MASTER_NUM];
      master_lite[i]->ACE_GEN_RATE_CACHE  = ACE_GEN_RATE_CACHE[i+smpl_cfg::FULL_MASTER_NUM];
      master_lite[i]->stop_gen(stop_gen);
    
      master_lite[i]->clk(clk);
      master_lite[i]->rst_n(rst_n);
    
      for(int j=0; j<smpl_cfg::SLAVE_NUM; ++j) {
        master_lite[i]->addr_map[j][0](addr_map[j][0]);
        master_lite[i]->addr_map[j][1](addr_map[j][1]);
      }
      
      master_lite[i]->ar_out(master_rd_req[i+smpl_cfg::FULL_MASTER_NUM]);
      master_lite[i]->r_in(master_rd_resp[i+smpl_cfg::FULL_MASTER_NUM]);
    
      master_lite[i]->aw_out(master_wr_req[i+smpl_cfg::FULL_MASTER_NUM]);
      master_lite[i]->w_out(master_wr_data[i+smpl_cfg::FULL_MASTER_NUM]);
      master_lite[i]->b_in(master_wr_resp[i+smpl_cfg::FULL_MASTER_NUM]);
    
      // Connect masters to IC
      interconnect.ar_in[i+smpl_cfg::FULL_MASTER_NUM](master_rd_req[i+smpl_cfg::FULL_MASTER_NUM]);
      interconnect.r_out[i+smpl_cfg::FULL_MASTER_NUM](master_rd_resp[i+smpl_cfg::FULL_MASTER_NUM]);
    
      interconnect.aw_in[i+smpl_cfg::FULL_MASTER_NUM](master_wr_req[i+smpl_cfg::FULL_MASTER_NUM]);
      interconnect.w_in[i+smpl_cfg::FULL_MASTER_NUM](master_wr_data[i+smpl_cfg::FULL_MASTER_NUM]);
      interconnect.b_out[i+smpl_cfg::FULL_MASTER_NUM](master_wr_resp[i+smpl_cfg::FULL_MASTER_NUM]);
    }
    

    // ToDO : Decide the LLC Interface
    for (int i=0; i<smpl_cfg::SLAVE_NUM; ++i) {
      // SLAVE
      slave[i]->sb_lock      = &sb_lock;      // Scoreboard by Ref
      slave[i]->sb_rd_req_q  = &sb_rd_req_q;  // Scoreboard by Ref
      slave[i]->sb_rd_resp_q = &sb_rd_resp_q; // Scoreboard by Ref
      
      slave[i]->sb_wr_req_q  = &sb_wr_req_q;  // Scoreboard by Ref
      slave[i]->sb_wr_data_q = &sb_wr_data_q; // Scoreboard by Ref
      slave[i]->sb_wr_resp_q = &sb_wr_resp_q; // Scoreboard by Ref
      
      slave[i]->AXI_STALL_RATE_RD = AXI_STALL_RATE_RD;
      slave[i]->AXI_STALL_RATE_WR = AXI_STALL_RATE_WR;
      slave[i]->SLAVE_ID      = i;
      slave[i]->stop_gen(stop_gen);
      slave[i]->clk(clk);
      slave[i]->rst_n(rst_n);
  
      for(int j=0; j<smpl_cfg::SLAVE_NUM; ++j) {
        slave[i]->addr_map[j][0](addr_map[j][0]);
        slave[i]->addr_map[j][1](addr_map[j][1]);
      }
      
      slave[i]->ar_in(slave_rd_req[i]);
      slave[i]->r_out(slave_rd_resp[i]);
      
      slave[i]->aw_in(slave_wr_req[i]);
      slave[i]->w_in(slave_wr_data[i]);
      slave[i]->b_out(slave_wr_resp[i]);
      
      // Slave Side
      interconnect.addr_map[i][0](addr_map[i][0]);
      interconnect.addr_map[i][1](addr_map[i][1]);
      
      interconnect.ar_out[i](slave_rd_req[i]);
      interconnect.r_in[i]  (slave_rd_resp[i]);
      
      interconnect.aw_out[i](slave_wr_req[i]);
      interconnect.w_out[i](slave_wr_data[i]);
      interconnect.b_in[i](slave_wr_resp[i]);
    }
    
    // BINDING - END
    std::cout << "---  Binding Succeed  ---\n";
    std::cout.flush();
  
    //sc_object_tracer<sc_clock> trace_clk(clk);
    Connections::set_sim_clk(&clk);
    
    SC_THREAD(harness_job);
    sensitive << clk.posedge_event();
    //sc_object_tracer<sc_clock> trace_clk(clk);
    
  } // End of Constructor
  
  void harness_job() {
    std::cout << "--- Simulation is Starting @" << sc_time_stamp() << " ---\n";

    std::cout.flush();
    rst_n.write(false);
    stop_gen.write(true);
    wait(CLK_PERIOD*2, SC_NS);
  
    rst_n.write(true);
    wait(CLK_PERIOD*2, SC_NS);
    
    stop_gen.write(false);
    wait(CLK_PERIOD*GEN_CYCLES, SC_NS);
    
    stop_gen.write(true);
    std::cout << "--- Transaction Generation Stopped @" << sc_time_stamp() << " ---\n";
    std::cout.flush();
    
    // Drain
    bool all_drained = false;
    do {
      int rd_req_remain  = 0;
      int rd_resp_remain = 0;
      
      for(int i=0; i<smpl_cfg::SLAVE_NUM;  ++i) rd_req_remain  += sb_rd_req_q[i].size();
      for(int i=0; i<smpl_cfg::ALL_MASTER_NUM; ++i) rd_resp_remain += sb_rd_resp_q[i].size();
      
      int wr_req_remain  = 0;
      int wr_data_remain = 0;
      int wr_resp_remain = 0;
      
      for(int i=0; i<smpl_cfg::SLAVE_NUM;++i)  wr_req_remain  += sb_wr_req_q[i].size();
      for(int i=0; i<smpl_cfg::SLAVE_NUM;++i)  wr_data_remain += sb_wr_data_q[i].size();
      for(int i=0; i<smpl_cfg::ALL_MASTER_NUM;++i) wr_resp_remain += sb_wr_resp_q[i].size();
      
      
      // Outstanding coherent transactions
      int cache_remain = 0;
      for (int i=0; i<smpl_cfg::FULL_MASTER_NUM; ++i) {
        for (auto it = master[i]->cache_outstanding.begin(); it != master[i]->cache_outstanding.end(); it++) {
          cache_remain += it->second;
          //std::cout << "Master " << i << " Addr:" << std::hex << it->first << std::dec << " : " << it->second << "\n";
        }
      }
      
      all_drained = (!rd_req_remain) && (!rd_resp_remain) && 
                    (!wr_req_remain) && (!wr_data_remain) && (!wr_resp_remain) &&
                    (!cache_remain);
      
      if(all_drained) {
        std::cout << "--- Everything Drained @" << sc_time_stamp() << " ---\n";
      } else {
        std::cout << "--- Wait to drain";
        std::cout << " (RD_Req: " << rd_req_remain << ", RD_Resp: " << rd_resp_remain << ")";
        std::cout << " (WR_Req: " << wr_req_remain << ", WR_Data: " << wr_data_remain << ", WR_Resp: "<< wr_resp_remain <<")";
        std::cout << " @" << sc_time_stamp() << " ---\n";
        
        /* DEBUG ....
        std::cout << "M0 AW_in available: " << (*master_wr_req)[0].num_available() << "\n"; // interconnect.aw_in_0.num_available() << "\n";
        std::cout << "M0 avail :";
        for (int i=0; i<5; ++i) std::cout << " " << interconnect.master_if_0.wr_reord_avail[i];
       // std::cout << "\n";
       // std::cout << __VERSION__ << "\n";
        */
        wait(CLK_PERIOD*DRAIN_CYCLES, SC_NS);
      }
      std::cout.flush();
    } while(!all_drained);
    
    std::cout << "--- Harness Exits @" << sc_time_stamp() << "\n";
    
    std::cout << "--- Simulation FINISHED @" << sc_time_stamp() << " ---\n";
    std::cout.flush();
    
    //--- Check for Errors ---//
    int err_sb_rd_req_not_found=0, err_sb_rd_resp_not_found=0;
    int err_sb_wr_req_not_found=0, err_sb_wr_data_not_found=0, err_sb_wr_resp_not_found=0;
    
    int rd_req_generated=0, rd_resp_generated=0;
    int rd_req_injected=0 , rd_req_ejected=0;
    int rd_resp_injected=0, rd_resp_ejected=0;
    
    int wr_req_generated=0,  wr_data_generated=0, wr_resp_generated=0;
    int wr_req_injected=0 ,  wr_req_ejected=0;
    int wr_data_injected=0 , wr_data_ejected=0;
    int wr_resp_injected=0,  wr_resp_ejected=0;
  
    for (int i=0; i<smpl_cfg::ALL_MASTER_NUM; ++i){
      // READS
      rd_req_generated  += master[i]->rd_trans_generated;
      rd_resp_generated += master[i]->rd_data_generated;
  
      rd_req_injected += master[i]->rd_trans_inj;
  
      rd_resp_injected += master[i]->rd_data_generated;
      rd_resp_ejected  += master[i]->rd_resp_ej;
  
      err_sb_rd_resp_not_found += master[i]->error_sb_rd_resp_not_found;
      // WRITES
      wr_req_generated  += master[i]->wr_trans_generated;
      wr_data_generated += master[i]->wr_data_generated;
  
      wr_req_injected  += master[i]->wr_trans_inj;
      wr_data_injected += master[i]->wr_data_inj;
  
      wr_resp_ejected  += master[i]->wr_resp_ej;
      
      err_sb_wr_resp_not_found += master[i]->error_sb_wr_resp_not_found;
    }
    
    for (int i=0; i<smpl_cfg::SLAVE_NUM; ++i){
      // READS
      rd_req_ejected  += slave[i]->rd_req_ej;
      
      err_sb_rd_req_not_found  += slave[i]->error_sb_rd_req_not_found;
      
      // WRITES
      wr_resp_generated += slave[i]->wr_resp_generated;
      
      wr_req_ejected   += slave[i]->wr_req_ej;
      wr_data_ejected  += slave[i]->wr_data_ej;
      
      wr_resp_injected += slave[i]->wr_resp_inj;
      
      err_sb_wr_req_not_found  += slave[i]->error_sb_wr_req_not_found;
      err_sb_wr_data_not_found += slave[i]->error_sb_wr_data_not_found;
    }
    
    bool error = (rd_req_injected - rd_req_ejected) || (rd_resp_injected - rd_resp_ejected) || err_sb_rd_req_not_found || err_sb_rd_resp_not_found;
    /*
    std::cout << "\n";
    if (error) {
      std::cout << "!!! --- FAILED --- !!!\n";
      
      std::cout << "READS  : " << (rd_req_injected-rd_req_ejected)   << " Reqs  Dropped\n";
      std::cout << "         " << (rd_resp_injected-rd_resp_ejected) << " Resps Dropped\n";
      std::cout << "         " << err_sb_rd_req_not_found            << " Reqs  Not Found in SB\n";
      std::cout << "         " << err_sb_rd_resp_not_found           << " Resps Not Found in SB\n";
      
      std::cout << "         " << " Out of :\n";
      std::cout << "         " << rd_req_generated      << " Reqs  Generated\n";
      std::cout << "         " << rd_resp_generated     << " Resps Generated\n";
      
      std::cout << "WRITES : " << (wr_req_injected-wr_req_ejected)   << " Reqs  Dropped\n";
      std::cout << "         " << (wr_data_injected-wr_data_ejected) << " Data  Dropped\n";
      std::cout << "         " << (wr_resp_injected-wr_resp_ejected) << " Resps Dropped\n";
      std::cout << "         " << err_sb_wr_req_not_found            << " Reqs  Not Found in SB\n";
      std::cout << "         " << err_sb_wr_data_not_found           << " Data  Not Found in SB\n";
      std::cout << "         " << err_sb_wr_resp_not_found           << " Resps Not Found in SB\n";
      
      std::cout << "         " << " Out of :\n";
      std::cout << "         " << wr_req_generated      << " Reqs  Generated\n";
      std::cout << "         " << wr_data_generated     << " Data  Generated\n";
      std::cout << "         " << wr_resp_generated     << " Resps Generated\n";
    } else {
      std::cout << "         " << rd_req_generated      << " RD_Reqs  Generated\n";
      std::cout << "         " << rd_resp_generated     << " RD_Resps Generated\n\n";
      
      std::cout << "         " << wr_req_generated      << " WR_Reqs  Generated\n";
      std::cout << "         " << wr_data_generated     << " WR_Data  Generated\n";
      std::cout << "         " << wr_resp_generated     << " WR_Resps Generated\n\n";
      std::cout << "PASSED. No Errors.\n";
    }
    std::cout << "\n";
    */
    std::cout.flush();
    
    // Print Masters' Caches
    for (int i=0; i<smpl_cfg::FULL_MASTER_NUM; ++i) {
      std::cout << "--- [ Master " << i+smpl_cfg::SLAVE_NUM << "] ---\n";
      for(auto it = master[i]->cache.begin(); it != master[i]->cache.end(); it++) {
        std::cout << "Addr:"<< std::hex << it->first << std::dec << " : " << it->second << "\n";
      }
      //std::cout << "\n";
    }
  
    for (int i=0; i<smpl_cfg::LITE_MASTER_NUM; ++i) {
      std::cout << "--- [ Master " << i+smpl_cfg::SLAVE_NUM+smpl_cfg::FULL_MASTER_NUM << "] ---\n";
      std::cout << "        ACE-Lite " << "\n";
    }
    
    // Delay calculation
    std::cout << "\n";
    unsigned long long int wr_delay_full_sum_glob = 0;
    unsigned long long int rd_delay_full_sum_glob = 0;
    unsigned long long int wr_trans_sum_glob      = 0;
    unsigned long long int rd_trans_sum_glob      = 0;
    unsigned long long int rd_data_count_glob     = 0;
    unsigned long long int wr_data_count_glob     = 0;
  
    unsigned long long int wr_delay_full_sum_p_m[smpl_cfg::ALL_MASTER_NUM];
    unsigned long long int rd_delay_full_sum_p_m[smpl_cfg::ALL_MASTER_NUM];
    unsigned long long int wr_trans_sum_p_m[smpl_cfg::ALL_MASTER_NUM];
    unsigned long long int rd_trans_sum_p_m[smpl_cfg::ALL_MASTER_NUM];
    for(int i=0; i<smpl_cfg::ALL_MASTER_NUM; ++i) {
      wr_delay_full_sum_p_m[i] = 0; rd_delay_full_sum_p_m[i] = 0;
      wr_trans_sum_p_m[i]      = 0; rd_trans_sum_p_m[i]      = 0;
    }
    
    
    for (int i=0; i<smpl_cfg::ALL_MASTER_NUM; ++i) {
      rd_delay_full_sum_p_m[i] += master[i]->rd_resp_delay;
      rd_trans_sum_p_m[i]      += master[i]->rd_resp_count;
      
      rd_delay_full_sum_glob += master[i]->rd_resp_delay;
      rd_trans_sum_glob      += master[i]->rd_resp_count;
      
      rd_data_count_glob += master[i]->rd_resp_data_count;
      
      wr_delay_full_sum_p_m[i] += master[i]->wr_resp_delay;
      wr_trans_sum_p_m[i]      += master[i]->wr_resp_count;
      
      wr_delay_full_sum_glob += master[i]->wr_resp_delay; 
      wr_trans_sum_glob      += master[i]->wr_resp_count;
      
      wr_data_count_glob += master[i]->wr_resp_data_count;
    }
    
    sc_time this_clk_period = clk.period();
    unsigned long long int total_cycles = sc_time_stamp() / this_clk_period;
    /*
    std::cout << "Delay Per Master Slave(delay, Throughput) :\n";
    for (int i=0; i<smpl_cfg::MASTER_NUM; i++) {
      std::cout << "M" << i << " RD: " << (rd_trans_sum_p_m[i] ? ((float)rd_delay_full_sum_p_m[i] / (float)rd_trans_sum_p_m[i]) : 0)
                                       << ", "
                                       << (rd_trans_sum_p_m[i] ? ((float)master[i]->rd_resp_data_count / (float)master[i]->last_rd_sinked_cycle) : 0)
                                       << "\n   WR: "
                                       << (wr_trans_sum_p_m[i] ? ((float)wr_delay_full_sum_p_m[i] / (float)wr_trans_sum_p_m[i]) : 0)
                                       << ", "
                                       << (wr_trans_sum_p_m[i] ? ((float)master[i]->wr_resp_data_count / (float)total_cycles) : 0)
                                       << "\n";
    }
    
    float rd_delay_full_total     = ((float)rd_delay_full_sum_glob / (float)rd_trans_sum_glob);
    float wr_delay_full_total     = ((float)wr_delay_full_sum_glob / (float)wr_trans_sum_glob);
    
    float rd_throughput_total     = ((float)rd_data_count_glob      / (float)total_cycles) / (float)smpl_cfg::MASTER_NUM;
    float wr_throughput_total     = ((float)wr_data_count_glob      / (float)total_cycles) / (float)smpl_cfg::MASTER_NUM;
    
    std::cout << "                               (RD, WR) \n";
    std::cout << "Full     Avg delay(cycles)      : " << rd_delay_full_total << ", "<< wr_delay_full_total << "\n";
    std::cout << "Throughput   (flits/cycle/node) : " << rd_throughput_total << ", "<< wr_throughput_total << "\n";
    */
    std::cout << __VERSION__ << "\n";
    std::cout.flush();
    
    std::cout << "\n Simulation Finished! \n";
    
    sc_stop();
  }
}; // End of harness

#endif // ACE_IC_HARNESS_H
