NoCpad
========

HLS implementation of a modular AXI interconnect.

AXI Master and Slave interfaces convert AXI traffic into a packetized format that is routed through an Network-on-Chip(NoC). A NoC is comprised by routers that provide communication between the various interfaces.

Low level communication between components are based on the Connections channel implementation, included with the Catapult HLS tool [HLSLibs](https://github.com/hlslibs/matchlib_connections), and MatchLib, a SystemC/C++ library of commonly-used hardware functions and components [MatchLib](https://github.com/NVlabs/matchlib)

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
* `src/fifo/fifo_queue_oh.h` An onehot FIFO implementation

Routers:
* `src/router/router_wh_buf.h` Wormhole router implementation using Matchlib In-OutBuffer ports
* `src/router/router_wh_fifo.h` Wormhole router implementation using custom onehot FIFO implementation
* `src/router/credits/router_vc.h` Virtual Channel based router using a credit based flow control and architecture similar to combined allocation of [Microarchitecture of Network-on-Chip Routers](https://www.springer.com/gp/book/9781461443001)

Interfaces:
* `src/axi_ifs/axi_master_if.h` Master interface that connects the Master agent to the network
* `src/axi_ifs/axi_master_if_reord.h` Master interface with transaction reordering capabilities
* `src/axi_ifs/axi_slave_if.h` Slave interface that connects the Slave agent to the network

* `src/axi_ifs/credits/axi_master_if.h` Master interface that implements credit flow control to be used in a Virtual Channel based Network-on-Chip
* `src/axi_ifs/credits/axi_master_if_reord.h` Master interface with credit flow control and transaction reorder capabilities
* `src/axi_ifs/credits/axi_slave_if.h` Slave interface that implements credit flow control to be used in a Virtual Channel based Network-on-Chip

Testbench files:
* `tb/tb_axi_con/axi_master.h` Testbench component that generates Requests and verifies the responses
* `tb/tb_axi_con/axi_slave.h` Testbench component that consumes AXI and verifies received Requests and produces AXI responses
* `tb/tb_axi_con/harness.h` Testbench component that parameterizes and setups the necessary testbench master-slave agents and connects the underlying DUT AXI interconnect.

# Examples 

* `examples/axi-ic_2d-mesh_1dst/ic_top_2d.h` 2 Master-2 Slave 64bit AXI interconnect with two separate 2-D mesh Request-Response networks. The ordering scheme allows outstanding transaction only towards the same destination, to follow ordering requirements.
* `examples/axi-ic_2d-mesh_multi-dst/ic_top_2d.h` Same network architecture. The ordering scheme allows outstanding to multiple destinations for transactions of different IDs, followiong AXI ordering requirements.
* `examples/axi-ic_2d-mesh_reord/ic_top_2d.h` Same network architecture. The ordering scheme allows outstanding even where transaction reordering is possible, but utilizes a reordering buffer to guarantee AXI compatible ID order.

* `examples/credit_axi-ic_2d-mesh_multi-dst/ic_top_2d_1noc.h` 2 Master-2 Slave 64bit AXI interconnect with a single 2-D mesh with separate Virtual Channels for Requests and Responses to avoid deadlocks. The ordering scheme is that of multiple destinations.

* `examples/*/axi_main_con.cpp` Simply matches the examples architecture with the testbench harness

# Questions and Contributions


# Contributors


# Attribution

# License

NoCpad is licensed under the [MIT License](./LICENSE).

