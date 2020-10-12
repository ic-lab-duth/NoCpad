CC = g++

INCDIR ?=
INCDIR += -I. -I$(SYSTEMC_HOME)/include -I$(BOOST_HOME)/include -I$(CATAPULT_HOME)/Mgc_home/shared/include -I$(MATCHLIB_HOME)/cmod/include


LIBDIR ?=
LIBDIR += -L. -L$(SYSTEMC_HOME)/lib-linux64 -L$(BOOST_HOME)/lib

CFLAGS ?= 
CFLAGS += -Wall -Wno-unknown-pragmas $(INCDIR) $(LIBDIR)

HLS_CATAPULT ?= 1
ifeq ($(HLS_CATAPULT),1)
  CFLAGS += -DHLS_CATAPULT
endif

LIBS ?=
LIBS += -lstdc++ -lsystemc -lm -lpthread -lboost_timer -lboost_chrono -lboost_system

# SIM_MODE
# 0 = Synthesis view of Connections port and combinational code.
# 	This option can cause failed simulations due to SystemC's timing model.
# 1 = Cycle-accurate view of Connections port and channel code, CONNECTIONS_ACCURATE_SIM. (default)
# 2 = Faster TLM view of Connections port and channel code, CONNECTIONS_FAST_SIM.
SIM_MODE ?= 1
ifeq ($(SIM_MODE),1)
	USER_FLAGS += -DCONNECTIONS_ACCURATE_SIM -DSC_INCLUDE_DYNAMIC_PROCESSES
endif
ifeq ($(SIM_MODE),2)
	USER_FLAGS += -DCONNECTIONS_FAST_SIM -DSC_INCLUDE_DYNAMIC_PROCESSES
endif

# RAND_STALL
# 0 = Random stall of ports and channels disabled (default)
# 1 = Random stall of ports and channels enabled
#   This feature aids in latency insensitive design verication.
#   Note: Only valid if SIM_MODE = 1 (accurate) or 2 (fast)
ifeq ($(RAND_STALL),1)
	USER_FLAGS += -DCONN_RAND_STALL
endif

.PHONY: Build
Build: all


CFLAGS += -O0 -g -std=c++11 

all: sim_sc

LIBDIR += -L$(SYSTEMC_HOME)/lib -L$(BOOST_HOME)/stage/lib

USER_FLAGS += -DSC_INCLUDE_DYNAMIC_PROCESSES -DCONNECTIONS_ACCURATE_SIM

USER_FLAGS += -DUSE_ROUTER_ST_BUF

run:
	./sim_sc

sim_sc: $(wildcard ../../src/include/*.h) $(wildcard ../../src/axi_ifs/*.h) $(wildcard ../../src/routers/*.h)
	$(CC) -o sim_sc $(CFLAGS) $(USER_FLAGS) ./axi_main.cpp $(BOOSTLIBS) $(LIBS)

clean: sim_clean

sim_clean:
	rm -rf *.o sim_* out.wlf trace.vcd transcript Cata* design_check* *.vhd cata*log sim_* trace.vcd out.wlf transcript
