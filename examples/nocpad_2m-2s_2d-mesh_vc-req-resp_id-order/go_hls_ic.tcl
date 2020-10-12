solution new -state initial
solution options defaults
flow package require /SCVerify
solution options set /Output/PackageOutput false

## Use fsdb file for power flow - make sure your environment var $NOVAS_INST_DIR has been set before you launch Catapult.
solution options set /Flows/LowPower/SWITCHING_ACTIVITY_TYPE fsdb
## SCVerify settings
solution options set /Flows/SCVerify/USE_MSIM false
solution options set /Flows/SCVerify/USE_OSCI false
solution options set /Flows/SCVerify/USE_VCS true
solution options set /Flows/VCS/VCS_HOME $env(VCS_HOME)
if { [info exist env(VG_GNU_PACKAGE)] } {
    solution options set /Flows/VCS/VG_GNU_PACKAGE $env(VG_GNU_PACKAGE)
} else {
    solution options set /Flows/VCS/VG_GNU_PACKAGE $env(VCS_HOME)/gnu/linux
}
solution options set /Flows/VCS/VG_ENV64_SCRIPT source_me.csh
solution options set /Flows/VCS/SYSC_VERSION 2.3.1

# Verilog/VHDL
solution options set Output OutputVerilog true
solution options set Output/OutputVHDL false
# Reset FFs
solution options set Architectural/DefaultResetClearsAllRegs yes

# General constrains. Please refer to tool ref manual for detailed descriptions.
directive set -DESIGN_GOAL area
directive set -SPECULATE true
directive set -MERGEABLE true
directive set -REGISTER_THRESHOLD 256
directive set -MEM_MAP_THRESHOLD 32
directive set -FSM_ENCODING none
directive set -REG_MAX_FANOUT 0
directive set -NO_X_ASSIGNMENTS true
directive set -SAFE_FSM false
directive set -REGISTER_SHARING_LIMIT 0
directive set -ASSIGN_OVERHEAD 0
directive set -TIMING_CHECKS true
directive set -MUXPATH true
directive set -REALLOC true
directive set -UNROLL no
directive set -IO_MODE super
directive set -REGISTER_IDLE_SIGNAL false
directive set -IDLE_SIGNAL {}
directive set -TRANSACTION_DONE_SIGNAL true
directive set -DONE_FLAG {}
directive set -START_FLAG {}
directive set -BLOCK_SYNC none
directive set -TRANSACTION_SYNC ready
directive set -DATA_SYNC none
directive set -RESET_CLEARS_ALL_REGS yes
directive set -CLOCK_OVERHEAD 20.000000
directive set -OPT_CONST_MULTS use_library
directive set -CHARACTERIZE_ROM false
directive set -PROTOTYPE_ROM true
directive set -ROM_THRESHOLD 64
directive set -CLUSTER_ADDTREE_IN_WIDTH_THRESHOLD 0
directive set -CLUSTER_OPT_CONSTANT_INPUTS true
directive set -CLUSTER_RTL_SYN false
directive set -CLUSTER_FAST_MODE false
directive set -CLUSTER_TYPE combinational
directive set -COMPGRADE fast
directive set -PIPELINE_RAMP_UP true


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
set TB_FILES [list ./axi_main.cpp]

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
