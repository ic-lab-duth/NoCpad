### Header files
- `src/include/arbiters.h` HLS implementation of various arbitration schemes
- `src/include/axi4_configs_extra.h` Expansion of Matclib's AXI configuration
- `src/include/dnp20_axi.h` definitions of packetization structure
- `src/include/duth_fun.h` helper low-level HLS functions commonly used
- `src/include/flit_axi.h` Network flit class that transports AXI
- `src/include/onehot.h` Onehot wrapped class to introduce onehot representation  
- `src/include/fifo_queue_oh.h` An onehot FIFO implementation

### Routers
- `src/router_wh.h` Wormhole router implementation
- `src/router_vc.h` Virtual Channel based router similar to combined allocation paradigm of [Microarchitecture of Network-on-Chip Routers](https://www.springer.com/gp/book/9781461443001)

### AMBA AXI4 Interfaces:
- `src/axi_master_if.h` Master interface that connects the Master agent to the network, capable of multiple outstanding transactions under two schemes, towards the same transaction destination, and towards multiple detinations for transactions of different IDs
- `src/axi_master_if_reord.h` Master interface that connects the Master agent to the network, with out-of-order outstanding requests and reordering capabilities to maintain AXI ordering
- `src/axi_slave_if.h` Slave interface that connects the Slave agent to the network

- `src/axi_master_if_vc.h` Master interface that connects the Master agent to the network, capable of multiple outstanding transactions under two schemes. Supports Virtual Channels.
- `src/axi_master_if_vc_reord.h` Master interface that connects the Master agent to the network, with reordering capabilities and Virtual Channel based Network-on-Chip support.
- `src/axi_slave_if_vc.h` Slave interface that connects the Slave agent to the network that supports Virtual Channel based Network-on-Chip

### AMBA ACE Interfaces:
- `src/ace/ace_home.h` HOME node receives read and write coherent requests to be Serialized and impose a total ordering. When a request is received, it creates and sends the appropriate snoop requests to the necessary masters. Depending on the Snoop responses, either a reponse is sent to the initiator or data are requested from/to the main memory.
  - HOME node expects data in a single flit. Thus an entire cache line must fit in a flit.
  - Only a single coherent request is processed at any time. The next transaction starts after the ACK for the precious transaction is received. 

- `src/ace/ace_master_if.h` Master interface that connects a cached Master agent to the network. ACE master operates as a typical AXI interface and extends the requests with extra fields. Master interfaces routes any coherent transaction to the HOME node, to which also sends an ACK when a transaction has finished at its initiator.
  - The Snoop Data channel is sized to full cacheline. Thus only INCR bursts of length 0 are expected.
  - Barrier coherent requests are not implemented.

Furthermore the ACE master interface implements the extra ACE channels which apply snoop requests to the Master cache (i.e. its cache controller) for data and privilege exchange, receive the appropriate response and optionally data and sends that snoop response to HOME for further handling.

- `src/ace/acelite_master_if.h` Master interface that implements the ACE-Lite version of ACE, applicable to un-cached masters that need to access data within the shared region of the Full ACE Agents.

- `src/ace/ace_slave_if.h` Slave interface is a typical AXI Slave interface with minimal changes to be able to handle ACE DNP flits.
