NoCpad
========

HLS implementation of a modular AXI interconnect.

AXI Master and Slave interfaces convert AXI traffic into a packetized format that is routed through an Network-on-Chip(NoC). A NoC is comprised by routers that provide communication between the various interfaces.

Low level communication between components are based on the Connections channel implementation, included with the Catapult HLS tool [HLSLibs](https://github.com/hlslibs/matchlib_connections), and MatchLib, a SystemC/C++ library of commonly-used hardware functions and components [MatchLib](https://github.com/NVlabs/matchlib)

NoCpad targets to an agile interconnect devolepment, that may change during SoC design flow with different features, topologies, and protocol attributes. To stich an interconnect NoCpad provides highly parameterizable AXI interfaces of various features, routers for wormhole and Virtual Channel based interconnects to form arbitrary topologies, and arbiter library of various arbitration schemes.

AXI interfaces :
* AXI porotocol compatibility 
* Matchlib AXI definitions for easy interoperability
* All AXI burst types
* Multiple AXI TIDs
* Arbitrary AXI data lane widths, even under the same interconnect
* Narrow and unaligned transactions
* Internal packet based transport protocol (Duth Network Protocol, DNP)
* Arbitrary internal NoC widths

Routers :
* Configurable Input/Output ports
* Various arbitration schemes
* Able to form arbitrary network topologies
* Wormohole routers with connections Ready-Valid flow controll
* Virtual Channel based routers over connections links

Arbitration Library:
* Fixed Priority
* Round Robin
* Matrix arbiter 
* Weighted Round Robin
* Deficit Round Robin

# Getting Started

## Tool versions

The codebase is based on Connections and MatchLib, and follows the similar tool/dependency versions:

* `gcc` - 4.9.3 (with C++11)
* `systemc` - 2.3.1
* `boost` - 1.68.0
* `catapult` - 10.5a
*  connections - included with catapult
* `QuestaSim` - 2019.3_1

## Environment requirements

Makefiles and synthesis scripts expect the following environment variables:
(A sample bash script can be found in examples/setenv.sh)

* `CATAPULT_HOME`
* `SYSTEMC_HOME`
* `BOOST_HOME`
* `MATCHLIB_HOME`

In addition, the boost and systemc library locations are expected to be in `LD_LIBRARY_PATH`.

## Build and run

### C++ compile and simulate
    cd examples/<example>
    make
    ./sim_sc 

### Mentor Catapult synthesis
    cd examples/<example>
    catapult -file go_hls_ic.tcl

# Directory structure
Header files:
* `src/include/arbiters.h` HLS implementation of various arbitration schemes
* `src/include/axi4_configs_extra.h` Expansion of Matclib's AXI configuration
* `src/include/dnp20_axi.h` definitions of packetization structure
* `src/include/duth_fun.h` helper low-level HLS functions commonly used
* `src/include/flit_axi.h` Network flit class that transports AXI
* `src/include/onehot.h` Onehot wrapped class to introduce onehot representation  
* `src/include/fifo_queue_oh.h` An onehot FIFO implementation

Routers:
* `src/router_wh.h` Wormhole router implementation
* `src/router_vc.h` Virtual Channel based router similar to combined allocation paradigm of [Microarchitecture of Network-on-Chip Routers](https://www.springer.com/gp/book/9781461443001)

Interfaces:
* `src/axi_master_if.h` Master interface that connects the Master agent to the network, capable of multiple outstanding transactions under two schemes, towards the same transaction destination, and towards multiple detinations for transactions of different IDs
* `src/axi_master_if_reord.h` Master interface that connects the Master agent to the network, with out-of-order outstanding requests and reordering capabilities to maintain AXI ordering
* `src/axi_slave_if.h` Slave interface that connects the Slave agent to the network

* `src/axi_master_if_vc.h` Master interface that connects the Master agent to the network, capable of multiple outstanding transactions under two schemes. Supports Virtual Channels.
* `src/vc_based/axi_master_if_vc_reord.h` Master interface that connects the Master agent to the network, with reordering capabilities and Virtual Channel based Network-on-Chip support.
* `src/axi_slave_if_vc.h` Slave interface that connects the Slave agent to the network that supports Virtual Channel based Network-on-Chip

Testbench files:
* `tb/tb_axi_con/axi_master.h` Testbench component that generates diverse Requests and verifies the responses
* `tb/tb_axi_con/axi_slave.h` Testbench component that consumes and verifies received AXI Requests and produces AXI responses
* `tb/tb_axi_con/harness.h` Testbench component that parameterizes and setups the necessary testbench master-slave agents and connects the underlying DUT AXI interconnect.

# Examples 

* `examples/nocpad_2m-2s_2d-mesh_basic-order/ic_top_2d.h` 2 Master-2 Slave 64bit AXI interconnect with two separate 2-D mesh Request-Response networks. The ordering scheme allows outstanding transaction only towards the same destination, to follow ordering requirements.
* `examples/nocpad_2m-2s_2d-mesh_id-order/ic_top_2d.h` Same network architecture. The ordering scheme allows outstanding to multiple destinations for transactions of different IDs, followiong AXI ordering requirements.
* `examples/nocpad_2m-2s_2d-mesh_reorder/ic_top_2d.h` Same network architecture. The ordering scheme allows outstanding even where transaction reordering is possible, but utilizes a reordering buffer to guarantee AXI compatible ID order.

* `examples/nocpad_2m-2s_2d-mesh_vc-req-resp_id-order/ic_top_2d_1noc.h` 2 Master-2 Slave 64bit AXI interconnect with a single 2-D mesh with separate Virtual Channels for Requests and Responses to avoid deadlocks. The ordering scheme is that of multiple destinations.

* `examples/*/axi_main_con.cpp` Simply matches the examples architecture with the testbench harness

# Questions and Contributions


# Contributors
Dimitris Konstantinou,
Dionisis Filippas,
Giorgos  Dimitrakopoulos

# Attribution

# License

NoCpad is licensed under the [MIT License](./LICENSE).

