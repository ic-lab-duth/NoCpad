# NoCpad #

### High-Level-Synthesis-ready AMBA AXI-4 compliant Network on Chip ###

NoCpad provides optimized HLS-ready SystemC models of all required Network-on-Chip components, such as network interfaces and routers (including virtual channels), in order to build a scalable AMBA AXI-4 compliant SoC interconnect. Quality of results in terms of networking performance as well as hardware PPA matches closely that of custom RTL.

AXI Master and Slave interfaces convert AXI traffic into a packetized format that is routed through an Network-on-Chip(NoC). A NoC is comprised by routers that provide communication between the various interfaces. Low level communication between components is performed using Connection channels, included with Mentor's Catapult HLS [HLSLibs](https://github.com/hlslibs/matchlib_connections), and MatchLib, a SystemC/C++ library of commonly-used hardware functions and components [MatchLib](https://github.com/NVlabs/matchlib)

Why does it make sense to build a NoC using HLS?
- NoC can change many times during design flow 
    - Changes can be structural and/or architectural
    - Re-verification should be fast
- SystemC/C++ models abstract enough to be highly configurable
    - Complex customizations can be easily architected
    - C++ simulation is faster
- HLS constraints can produce the needed result
    - Or even lead to new unexplored alternatives 
    - No need for re-verification

NoCpad targets agile interconnect development. To stich a SoC interconnect NoCpad provides highly parameterizable AXI interfaces of various features, routers for wormhole and Virtual Channel based interconnects to form arbitrary topologies, and a library of various arbitration schemes.

AMBA-AXI4 interfaces :
- AXI porotocol compatibility 
    - All burst types
    - Multiple configurable transaction IDs
    - Arbitrary data lane widths, even under the same interconnect
    - Narrow and unaligned transactions
- Internal packet based transport protocol (Duth Network Protocol, DNP)
    - Arbitrary internal NoC widths
- Matchlib AXI definitions for easy interoperability

NoC Routers :
* Configurable number of Input/Output ports
* Various arbitration schemes
* Able to form arbitrary network topologies
* Two forms of link-level flow control
    * Wormhole routers with Ready-Valid flow controll using Connections
    * Virtual Channel based usin Connections and credit-based flow control

Library of arbiter components:
* Fixed Priority
* Round Robin
* Matrix arbiter 
* Weighted Round Robin
* Deficit Round Robin
* Merged arbiter multiplexers

## Getting Started ##

### Tool versions ###

The codebase is based on Connections and MatchLib, and follows the similar tool/dependency versions:

* `gcc` - 4.9.3 (with C++11)
* `systemc` - 2.3.1
* `boost` - 1.68.0
* `catapult` - 10.5a
*  connections - included with catapult
* `QuestaSim` - 2019.3_1

### Environment requirements

Makefiles and synthesis scripts expect the following environment variables:
(A sample bash script can be found in examples/setenv.sh)

* `CATAPULT_HOME`
* `SYSTEMC_HOME`
* `BOOST_HOME`
* `MATCHLIB_HOME`

In addition, the boost and systemc library locations are expected to be in `LD_LIBRARY_PATH`.

### C++ compile and simulate
    cd examples/<example>
    make
    ./sim_sc 

### Mentor Catapult synthesis
    cd examples/<example>
    catapult -file go_hls_ic.tcl

## Contributors
Dimitris Konstantinou, Giorgos  Dimitrakopoulos, Dionisis Filippas

## Pending additions
- Support for ACE cache coherent interfaces

## License
NoCpad is licensed under the [MIT License](./LICENSE).

