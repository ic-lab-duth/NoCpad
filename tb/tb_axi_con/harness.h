#ifndef AXI_IC_HARNESS_H
#define AXI_IC_HARNESS_H

#include "systemc.h"

#include <mc_scverify.h>

#define NVHLS_VERIFY_BLOCKS (ic_top)

#include "stdlib.h"
#include <string>

#include "../../tb/tb_axi_con/axi_master.h"
#include "../../tb/tb_axi_con/axi_slave.h"

#include <iostream>
#include <fstream>

SC_MODULE(harness) {
  const int CLK_PERIOD = 5;
  const int GEN_CYCLES = 2 * 1000;
  
  const int GEN_RATE_RD[smpl_cfg::MASTER_NUM] = {40, 40};
  const int GEN_RATE_WR[smpl_cfg::MASTER_NUM] = {40, 40};
  
  const int STALL_RATE_RD = 00;
  const int STALL_RATE_WR = 00;
  
  const int DRAIN_CYCLES = GEN_CYCLES/10;
  
  typedef typename axi::axi4<axi::cfg::standard_duth> axi4_;
  typedef typename axi::AXI4_Encoding            enc_;
   
  sc_clock        clk;
  sc_signal<bool> rst_n;
  
  sc_signal<bool> stop_gen;
  
  sc_signal< sc_uint<32> >  addr_map[smpl_cfg::SLAVE_NUM][2];
  
  // --- Scoreboards --- //
  // Scoreboards refer to the receiver of the queue.
  // I.e. the receiver checks what is expected to be received. Thus sender must take care to push Transactions to the appropriate queue
  sc_mutex                                                       sb_lock;
  std::vector< std::deque< msg_tb_wrap<axi4_::AddrPayload> > >   sb_rd_req_q;
  std::vector< std::deque< msg_tb_wrap<axi4_::ReadPayload> > >   sb_rd_resp_q;
  
  std::vector< std::deque< msg_tb_wrap<axi4_::AddrPayload> > >   sb_wr_req_q;
  std::vector< std::deque< msg_tb_wrap<axi4_::WritePayload> > >  sb_wr_data_q;
  std::vector< std::deque< msg_tb_wrap<axi4_::WRespPayload> > >  sb_wr_resp_q;
  
  axi_master<smpl_cfg::RD_LANES, smpl_cfg::RD_LANES, smpl_cfg::WR_LANES, smpl_cfg::WR_LANES, smpl_cfg::MASTER_NUM, smpl_cfg::SLAVE_NUM> *master[smpl_cfg::MASTER_NUM];
  axi_slave<smpl_cfg::RD_LANES, smpl_cfg::RD_LANES, smpl_cfg::WR_LANES, smpl_cfg::WR_LANES, smpl_cfg::MASTER_NUM, smpl_cfg::SLAVE_NUM>  *slave[smpl_cfg::SLAVE_NUM];
  
  CCS_DESIGN(ic_top) interconnect;

  // Master Side Channels
  Connections::Combinational<axi4_::AddrPayload>   *master_rd_req[smpl_cfg::MASTER_NUM];
  Connections::Combinational<axi4_::ReadPayload>   *master_rd_resp[smpl_cfg::MASTER_NUM];
  
  Connections::Combinational<axi4_::AddrPayload>   *master_wr_req[smpl_cfg::MASTER_NUM];
  Connections::Combinational<axi4_::WritePayload>  *master_wr_data[smpl_cfg::MASTER_NUM];
  Connections::Combinational<axi4_::WRespPayload>  *master_wr_resp[smpl_cfg::MASTER_NUM];
  
  // Slave Side Channels  
  Connections::Combinational<axi4_::AddrPayload>   *slave_rd_req[smpl_cfg::SLAVE_NUM];
  Connections::Combinational<axi4_::ReadPayload>   *slave_rd_resp[smpl_cfg::SLAVE_NUM];
  
  Connections::Combinational<axi4_::AddrPayload>   *slave_wr_req[smpl_cfg::SLAVE_NUM];
  Connections::Combinational<axi4_::WritePayload>  *slave_wr_data[smpl_cfg::SLAVE_NUM];
  Connections::Combinational<axi4_::WRespPayload>  *slave_wr_resp[smpl_cfg::SLAVE_NUM];
  
  SC_CTOR(harness) :
    clk("clock",10,SC_NS,0.5,0.0,SC_NS),
    rst_n("rst_n"),
    stop_gen("stop_gen"),
    
    sb_lock(),
    sb_rd_req_q(smpl_cfg::SLAVE_NUM),
    sb_rd_resp_q(smpl_cfg::MASTER_NUM),
    sb_wr_req_q(smpl_cfg::SLAVE_NUM),
    sb_wr_data_q(smpl_cfg::SLAVE_NUM),
    sb_wr_resp_q(smpl_cfg::MASTER_NUM),
    
    interconnect("interconnect")
  {
    
    addr_map[0][0] = 0;
    addr_map[0][1] = 0x0ffff;
    addr_map[1][0] = 0x10000;
    addr_map[1][1] = 0x2ffff;
    
    // Construct Components
    for (int i=0; i<smpl_cfg::MASTER_NUM; ++i) {
      master[i] = new axi_master<smpl_cfg::RD_LANES, smpl_cfg::RD_LANES, smpl_cfg::WR_LANES, smpl_cfg::WR_LANES, smpl_cfg::MASTER_NUM, smpl_cfg::SLAVE_NUM>(sc_gen_unique_name("master"));
      slave[i]  = new axi_slave <smpl_cfg::RD_LANES, smpl_cfg::RD_LANES, smpl_cfg::WR_LANES, smpl_cfg::WR_LANES, smpl_cfg::MASTER_NUM, smpl_cfg::SLAVE_NUM>(sc_gen_unique_name("slave"));
    
      master_rd_req[i]  = new Connections::Combinational<axi4_::AddrPayload>  (sc_gen_unique_name("master_rd_req"));
      master_rd_resp[i] = new Connections::Combinational<axi4_::ReadPayload>  (sc_gen_unique_name("master_rd_resp"));
                                      
      master_wr_req[i]  = new Connections::Combinational<axi4_::AddrPayload>   (sc_gen_unique_name("master_wr_req"));
      master_wr_data[i] = new Connections::Combinational<axi4_::WritePayload>  (sc_gen_unique_name("master_wr_data"));
      master_wr_resp[i] = new Connections::Combinational<axi4_::WRespPayload>  (sc_gen_unique_name("master_wr_resp"));
                                      
                                      
      // Slave Side Channels
      slave_rd_req[i]  = new Connections::Combinational<axi4_::AddrPayload>  (sc_gen_unique_name("slave_rd_req"));
      slave_rd_resp[i] = new Connections::Combinational<axi4_::ReadPayload>  (sc_gen_unique_name("slave_rd_resp"));
                                      
      slave_wr_req[i]  = new Connections::Combinational<axi4_::AddrPayload>   (sc_gen_unique_name("slave_wr_req"));
      slave_wr_data[i] = new Connections::Combinational<axi4_::WritePayload>  (sc_gen_unique_name("slave_wr_data"));
      slave_wr_resp[i] = new Connections::Combinational<axi4_::WRespPayload>  (sc_gen_unique_name("slave_wr_resp"));
    }
    
    std::cout << "---  Binding...  ---\n";
    std::cout.flush();
    
    // BINDING - START
    // MASTER
    for (int i=0; i<smpl_cfg::MASTER_NUM; ++i) {
      master[i]->sb_lock      = &sb_lock;      // Scoreboard by Ref
      master[i]->sb_rd_req_q  = &sb_rd_req_q;  // Scoreboard by Ref
      master[i]->sb_rd_resp_q = &sb_rd_resp_q; // Scoreboard by Ref
      
      master[i]->sb_wr_req_q  = &sb_wr_req_q;  // Scoreboard by Ref
      master[i]->sb_wr_data_q = &sb_wr_data_q; // Scoreboard by Ref
      master[i]->sb_wr_resp_q = &sb_wr_resp_q; // Scoreboard by Ref
      
      master[i]->MASTER_ID    = i;
      master[i]->GEN_RATE_RD  = GEN_RATE_RD[i];
      master[i]->GEN_RATE_WR  = GEN_RATE_WR[i];
      master[i]->stop_gen(stop_gen);
      
      master[i]->clk(clk);
      master[i]->rst_n(rst_n);
      for(int j=0; j<smpl_cfg::SLAVE_NUM; ++j) {
        master[i]->addr_map[j][0](addr_map[j][0]);
        master[i]->addr_map[j][1](addr_map[j][1]);
      }
      
      master[i]->ar_out(*master_rd_req[i]);
      master[i]->r_in(*master_rd_resp[i]);
      
      master[i]->aw_out(*master_wr_req[i]);
      master[i]->w_out(*master_wr_data[i]);
      master[i]->b_in(*master_wr_resp[i]);
      
      // SLAVE
      slave[i]->sb_lock      = &sb_lock;      // Scoreboard by Ref
      slave[i]->sb_rd_req_q  = &sb_rd_req_q;  // Scoreboard by Ref
      slave[i]->sb_rd_resp_q = &sb_rd_resp_q; // Scoreboard by Ref
      
      slave[i]->sb_wr_req_q  = &sb_wr_req_q;  // Scoreboard by Ref
      slave[i]->sb_wr_data_q = &sb_wr_data_q; // Scoreboard by Ref
      slave[i]->sb_wr_resp_q = &sb_wr_resp_q; // Scoreboard by Ref
      
      slave[i]->STALL_RATE_RD = STALL_RATE_RD;
      slave[i]->STALL_RATE_WR = STALL_RATE_WR;
      slave[i]->SLAVE_ID      = i;
      slave[i]->stop_gen(stop_gen);
      slave[i]->clk(clk);
      slave[i]->rst_n(rst_n);
      for(int j=0; j<smpl_cfg::SLAVE_NUM; ++j) {
        slave[i]->addr_map[j][0](addr_map[j][0]);
        slave[i]->addr_map[j][1](addr_map[j][1]);
      }
      
      slave[i]->ar_in(*slave_rd_req[i]);
      slave[i]->r_out(*slave_rd_resp[i]);
      
      slave[i]->aw_in(*slave_wr_req[i]);
      slave[i]->w_in(*slave_wr_data[i]);
      slave[i]->b_out(*slave_wr_resp[i]);
    }
    
    // IC-TOP
    interconnect.clk(clk);
    interconnect.rst_n(rst_n);
    for(int j=0; j<smpl_cfg::SLAVE_NUM; ++j) {
      interconnect.addr_map[j][0](addr_map[j][0]);
      interconnect.addr_map[j][1](addr_map[j][1]);
    }
    // Master Side
    for (int i=0; i<smpl_cfg::MASTER_NUM; ++i) {
      interconnect.ar_in[i](*master_rd_req[i]);
      interconnect.r_out[i](*master_rd_resp[i]);
  
      interconnect.aw_in[i](*master_wr_req[i]);
      interconnect.w_in[i](*master_wr_data[i]);
      interconnect.b_out[i](*master_wr_resp[i]);
    }
    // Slave Side
    for (int i=0; i<smpl_cfg::MASTER_NUM; ++i) {
      interconnect.ar_out[i](*slave_rd_req[i]);
      interconnect.r_in[i](*slave_rd_resp[i]);
  
      interconnect.aw_out[i](*slave_wr_req[i]);
      interconnect.w_out[i](*slave_wr_data[i]);
      interconnect.b_in[i](*slave_wr_resp[i]);
    }
    // BINDING - END
    std::cout << "---  Binding Succeed  ---\n";
    std::cout.flush();
    
    Connections::set_sim_clk(&clk);
    
    SC_THREAD(harness_job);
    sensitive << clk.posedge_event();
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
      for(int i=0; i<smpl_cfg::MASTER_NUM; ++i) rd_resp_remain += sb_rd_resp_q[i].size();
      
      int wr_req_remain  = 0;
      int wr_data_remain = 0;
      int wr_resp_remain = 0;
      
      for(int i=0; i<smpl_cfg::SLAVE_NUM;++i)  wr_req_remain  += sb_wr_req_q[i].size();
      for(int i=0; i<smpl_cfg::SLAVE_NUM;++i)  wr_data_remain += sb_wr_data_q[i].size();
      for(int i=0; i<smpl_cfg::MASTER_NUM;++i) wr_resp_remain += sb_wr_resp_q[i].size();
      
      all_drained = (!rd_req_remain) && (!rd_resp_remain) && 
                    (!wr_req_remain) && (!wr_data_remain) && (!wr_resp_remain);
      
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
    
    
    for (int i=0; i<smpl_cfg::MASTER_NUM; ++i){
      // READS
      rd_req_generated  += master[i]->rd_trans_generated;
      rd_resp_generated += master[i]->rd_data_generated;
      
      rd_req_injected += master[i]->rd_trans_inj;
      rd_req_ejected  += slave[i]->rd_req_ej;
      
      rd_resp_injected += master[i]->rd_data_generated;
      rd_resp_ejected  += master[i]->rd_resp_ej;
      
      err_sb_rd_req_not_found  += slave[i]->error_sb_rd_req_not_found;
      err_sb_rd_resp_not_found += master[i]->error_sb_rd_resp_not_found;
      
      // WRITES
      wr_req_generated  += master[i]->wr_trans_generated;
      wr_data_generated += master[i]->wr_data_generated;
      wr_resp_generated += slave[i]->wr_resp_generated;
      
      wr_req_injected  += master[i]->wr_trans_inj;
      wr_data_injected += master[i]->wr_data_inj;
      wr_req_ejected   += slave[i]->wr_req_ej;
      wr_data_ejected  += slave[i]->wr_data_ej;
      
      wr_resp_injected += slave[i]->wr_resp_inj;
      wr_resp_ejected  += master[i]->wr_resp_ej;
      
      err_sb_wr_req_not_found  += slave[i]->error_sb_wr_req_not_found;
      err_sb_wr_data_not_found += slave[i]->error_sb_wr_data_not_found;
      err_sb_wr_resp_not_found += master[i]->error_sb_wr_resp_not_found;
    }
    
    bool error = (rd_req_injected - rd_req_ejected) || (rd_resp_injected - rd_resp_ejected) || err_sb_rd_req_not_found || err_sb_rd_resp_not_found;
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
    std::cout.flush();
    
    // Delay calculation
    std::cout << "\n";
    unsigned long long int wr_delay_full_sum_glob = 0;
    unsigned long long int rd_delay_full_sum_glob = 0;
    unsigned long long int wr_trans_sum_glob      = 0;
    unsigned long long int rd_trans_sum_glob      = 0;
    unsigned long long int rd_data_count_glob     = 0;
    unsigned long long int wr_data_count_glob     = 0;
  
    unsigned long long int wr_delay_full_sum_p_m[smpl_cfg::MASTER_NUM];
    unsigned long long int rd_delay_full_sum_p_m[smpl_cfg::MASTER_NUM];
    unsigned long long int wr_trans_sum_p_m[smpl_cfg::MASTER_NUM];
    unsigned long long int rd_trans_sum_p_m[smpl_cfg::MASTER_NUM];
    for(int i=0; i<smpl_cfg::SLAVE_NUM; ++i) {
      wr_delay_full_sum_p_m[i] = 0; rd_delay_full_sum_p_m[i] = 0;
      wr_trans_sum_p_m[i]      = 0; rd_trans_sum_p_m[i]      = 0;
    }
    
    
    for (int i=0; i<smpl_cfg::MASTER_NUM; ++i) {
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
    
    std::cout << "\n";
    std::cout << __VERSION__ << "\n";
    std::cout.flush();
    
    sc_stop();
  }
}; // End of harness

#endif // AXI_IC_HARNESS_H
