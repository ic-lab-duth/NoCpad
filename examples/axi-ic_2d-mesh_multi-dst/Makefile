
include ../cmod_Makefile

CFLAGS += -O0 -g -std=c++11 

all: sim_sc

LIBDIR += -L$(SYSTEMC_HOME)/lib -L$(BOOST_HOME)/stage/lib

USER_FLAGS += -DSC_INCLUDE_DYNAMIC_PROCESSES -DCONNECTIONS_ACCURATE_SIM

USER_FLAGS += -DUSE_ROUTER_ST_BUF

run:
	./sim_sc

sim_sc: $(wildcard ../../src/include/*.h) $(wildcard ../../src/axi_ifs/*.h) $(wildcard ../../src/routers/*.h)
	$(CC) -o sim_sc $(CFLAGS) $(USER_FLAGS) ./axi_main_con.cpp $(BOOSTLIBS) $(LIBS)

clean: sim_clean

sim_clean:
	rm -rf *.o sim_* out.wlf trace.vcd transcript Cata* design_check* *.vhd cata*log sim_* trace.vcd out.wlf transcript
