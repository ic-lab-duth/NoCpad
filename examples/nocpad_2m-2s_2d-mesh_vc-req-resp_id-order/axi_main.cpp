#include "./ic_top_2d_1noc.h"
#include "../../tb/tb_axi_con/harness.h"

sc_trace_file* trace_file_ptr;

int sc_main(int argc, char *argv[]) {
  
  trace_file_ptr = sc_create_vcd_trace_file("trace");
  
  harness the_harness("the_harness");
  sc_start();  

  return (0);
  
}; // End of main
