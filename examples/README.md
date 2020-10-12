`examples/nocpad_2m-2s_2d-mesh_basic-order/ic_top_2d.h` 
2 Master-2 Slave 64bit AXI interconnect with two separate 2-D mesh Request-Response networks. 
The ordering scheme allows outstanding transaction only towards the same destination, 
to follow AXI transaction ordering requirements

`examples/nocpad_2m-2s_2d-mesh_id-order/ic_top_2d.h` 
Same network architecture. The ordering scheme allows outstanding to multiple destinations for transactions of 
different IDs, following AXI ordering requirements.

`examples/nocpad_2m-2s_2d-mesh_reorder/ic_top_2d.h` Same network architecture. 
The ordering scheme allows outstanding even where transaction reordering is possible, 
but utilizes a reordering buffer to guarantee AXI compatible ID order.

`examples/nocpad_2m-2s_2d-mesh_vc-req-resp_id-order/ic_top_2d_1noc.h` 
2 Master-2 Slave 64bit AXI interconnect with a single 2-D mesh with separate Virtual Channels for 
Requests and Responses to avoid deadlocks. The ordering scheme is that of multiple destinations.

`examples/nocpad_ACE-lite_2m-2mlite-2s_1stage/ic_top.h` 
2 ACE Master,2 ACE-Lite Master and 2 Slave 64bit AXI interconnect with a single HOME node for coherency management. ACE Masters maintain cached data and are getting snooped in case of coherent transactions, whereas ACE-lite masters only participate to access data without an internal cache thus there is no need to get snooped. The internal NoC uses separate 1stage Networks for traffic isolation(e.g. requests, responses, snoops requests) to avoid deadlocks. The Read, Write and Snoop data channels are sized according to the chosen configuration parameter for the cache line width.

`examples/nocpad_ACE_4m-2s_1stage/ic_top.h` 
4 ACE Master - 2 Slave 64bit AXI interconnect with a single HOME node for coherency management. The configuration is to the previous example, but instead only ACE masters participate for cache coherency.

`examples/*/axi_main.cpp` 
Simply matches the examples AXI architecture with the testbench harness

`examples/*/ace_main.cpp` 
Simply matches the examples ACE architecture with the testbench harness
