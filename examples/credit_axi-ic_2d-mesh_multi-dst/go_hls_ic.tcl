
source ../run_hls_global_setup.tcl
solution options set /Flows/SCVerify/USE_VCS false
solution options set /Flows/SCVerify/USE_MSIM true

options set Input/SearchPath ". $env(MATCHLIB_HOME)/cmod $env(MATCHLIB_HOME)/cmod/include $env(BOOST_HOME)/include"
options set Input/CppStandard c++11
options set Architectural/DesignGoal latency

#global variables across all steps
set TOP_NAME "ic_top"
set CLK_NAME clk
set CLK_PERIOD 10
set SRC_DIR "../../"

set DESIGN_FILES [list ./ic_top_2d_1noc.h]
set TB_FILES [list ./axi_main_con.cpp]

# Choose router
set ROUTER_SELECT_FLAG "-DUSE_ROUTER_ST_BUF"

if { [info exists env(HLS_CATAPULT)] && ($env(HLS_CATAPULT) eq "1") } {
  set HLS_CATAPULT_FLAG "-DHLS_CATAPULT"
} else {
  set HLS_CATAPULT_FLAG ""
}

solution options set Input/TargetPlatform x86_64

# Add your design here
foreach design_file $DESIGN_FILES {
	solution file add $design_file -type SYSTEMC
}
foreach tb_file $TB_FILES {
	solution file add $tb_file -type SYSTEMC -exclude true
}
options set Input/CompilerFlags "-DHLS_CATAPULT -DSC_INCLUDE_DYNAMIC_PROCESSES -DCONNECTIONS_ACCURATE_SIM $HLS_CATAPULT_FLAG $ROUTER_SELECT_FLAG"
go analyze
solution library add nangate-45nm_beh -- -rtlsyntool OasysRTL -vendor Nangate -technology 045nm
#solution library add mgc_sample-065nm-dw_beh_dc -- -rtlsyntool DesignCompiler -vendor Sample -technology 065nm -Designware Yes
#solution library add ram_sample-065nm-singleport_beh_dc

# Clock, interface constrain
set CLK_PERIODby2 [expr $CLK_PERIOD/2]
directive set -CLOCKS "$CLK_NAME \"-CLOCK_PERIOD $CLK_PERIOD -CLOCK_EDGE rising -CLOCK_UNCERTAINTY 0.0 -CLOCK_HIGH_TIME $CLK_PERIODby2 -RESET_SYNC_NAME rst -RESET_ASYNC_NAME arst_n -RESET_KIND sync -RESET_SYNC_ACTIVE high -RESET_ASYNC_ACTIVE low -ENABLE_NAME {} -ENABLE_ACTIVE high\"    "
directive set -CLOCK_NAME $CLK_NAME
directive set GATE_REGISTERS false

directive set -DESIGN_HIERARCHY "$TOP_NAME"

go compile
go libraries
go assembly

go architect
go allocate
go schedule
go dpfsm
go extract
#flow run /OasysRTL/launch_tool ./concat_rtl.v.or v
# go switching
project save

# exit
