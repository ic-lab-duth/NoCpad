#ifndef __FLIT_DNP_H__
#define __FLIT_DNP_H__

#include "systemc.h"
#include "nvhls_connections.h"

#include "./dnp20_axi.h"

#ifndef __SYNTHESIS__
	#include <string>
	#include <iostream>
#endif

//ToDo add this in thr flit class
enum FLIT_TYPE {HEAD=0 , BODY=1, TAIL=3, SINGLE=2};

template<unsigned char PHIT_NUM>
struct flit_dnp {
  sc_uint<2>  type;
  sc_uint<2>  vc;
  //sc_uint<32> dbg_id;
  sc_uint<dnp::PHIT_W> data[PHIT_NUM];
  
  static const int width = 2+2+(PHIT_NUM*dnp::PHIT_W); // Matchlib Marshaller requirement
  
  // helping functions to retrieve flit info (e.g. flit type, source, destination)
	inline bool performs_rc()   { return ((type == HEAD) || (type == SINGLE)); }
	inline bool resets_states() { return (type == TAIL); }
  
  inline bool is_head()   {return (type==HEAD);};
  inline bool is_tail()   {return (type==TAIL);};
  inline bool is_body()   {return (type==BODY);};
  inline bool is_single() {return (type==SINGLE);};
  
  // DNP fields set/getters
  inline sc_uint<dnp::D_W> get_dst()  const {return ((data[0] >> dnp::D_PTR) & ((1<<dnp::D_W)-1));};
  inline sc_uint<dnp::S_W> get_src()  const {return ((data[0] >> dnp::S_PTR) & ((1<<dnp::S_W)-1));};
  inline sc_uint<dnp::T_W> get_type() const {return ((data[0] >> dnp::T_PTR) & ((1<<dnp::T_W)-1));};
  inline sc_uint<dnp::V_W> get_vc()   const {return vc;};
  inline sc_uint<dnp::Q_W> get_qos()   const {return ((data[0] >> dnp::Q_PTR) & ((1<<dnp::Q_W)-1));};
  
  inline void set_dst(sc_uint<dnp::D_W>  dst ) { data[0] = (data[0].range(dnp::PHIT_W-1, dnp::D_PTR+dnp::D_W) << (dnp::D_PTR+dnp::D_W)) |
                                                                (dst  << dnp::D_PTR) |
                                                                (data[0].range(dnp::D_PTR-1, 0));
  };
  inline void set_src(sc_uint<dnp::S_W>  src ) { data[0] = (data[0].range(dnp::PHIT_W-1, dnp::S_PTR+dnp::S_W) << (dnp::S_PTR+dnp::S_W)) |
                                                                (src  << dnp::S_PTR) |
                                                                (data[0].range(dnp::S_PTR-1, 0));
  };
  inline void set_type(sc_uint<dnp::T_W> type) { data[0] = (data[0].range(dnp::PHIT_W-1, dnp::T_PTR+dnp::T_W) << (dnp::T_PTR+dnp::T_W)) |
                                                                (type  << dnp::T_PTR) |
                                                                (data[0].range(dnp::T_PTR-1, 0));
  };
  inline void set_vc(sc_uint<dnp::V_W>   vc_  ) { vc = vc_; };
  inline void set_qos(sc_uint<dnp::Q_W>  qos ) { data[0] = (data[0].range(dnp::PHIT_W-1, dnp::Q_PTR+dnp::Q_W) << (dnp::Q_PTR+dnp::Q_W)) |
                                                                (qos  << dnp::Q_PTR) |
                                                                (data[0].range(dnp::Q_PTR-1, 0));
  };
  inline void set_network(
          sc_uint<dnp::S_W>  src,
          sc_uint<dnp::D_W>  dst,
          sc_uint<dnp::V_W>  vc,
          sc_uint<dnp::T_W>  type,
          sc_uint<dnp::Q_W>  qos
  ) {
    set_vc(vc);
    data[0] = (data[0].range(dnp::PHIT_W-1, dnp::T_PTR+dnp::T_W) << (dnp::T_PTR+dnp::T_W)) |
              (type << dnp::T_PTR) |
              (qos  << dnp::Q_PTR) |
              (dst  << dnp::D_PTR) |
              (src  << dnp::S_PTR) ;
  };
  
  template<typename T>
  inline void set_rd_req (const T& rd_req, sc_uint<dnp::REORD_W>  reord_tct) {
    this->data[0] = ((sc_uint<dnp::PHIT_W>) reord_tct      << dnp::req::REORD_PTR) |
                    ((sc_uint<dnp::PHIT_W>) rd_req.id      << dnp::req::ID_PTR)    |
                    (sc_uint<dnp::PHIT_W>) this->data[0].range(dnp::T_PTR+dnp::T_W-1, 0); // Keep the network portion unaffected
  
    this->data[1] = ((sc_uint<dnp::PHIT_W>) rd_req.len            << dnp::req::LE_PTR) |
                    ((sc_uint<dnp::PHIT_W>)(rd_req.addr & 0xffff) << dnp::req::AL_PTR) ;
  
    this->data[2] = ((sc_uint<dnp::PHIT_W>) rd_req.burst                   << dnp::req::BU_PTR  ) |
                    ((sc_uint<dnp::PHIT_W>) rd_req.size                    << dnp::req::SZ_PTR  ) |
                    ((sc_uint<dnp::PHIT_W>)(rd_req.addr >> dnp::AL_W) << dnp::req::AH_PTR) ;
  };
  
  template<typename T>
  inline void set_wr_req (const T& wr_req, sc_uint<dnp::REORD_W>  reord_tct) {this->set_rd_req(wr_req, reord_tct);};
  
  template<typename T>
  inline void set_rd_resp (const T& rd_req, sc_uint<dnp::REORD_W>  reord_tct) {
    this->data[0] = ((sc_uint<dnp::PHIT_W>) rd_req.burst   << dnp::rresp::BU_PTR) |
                    ((sc_uint<dnp::PHIT_W>) reord_tct      << dnp::rresp::REORD_PTR) |
                    ((sc_uint<dnp::PHIT_W>) rd_req.id      << dnp::rresp::ID_PTR) |
                    (sc_uint<dnp::PHIT_W>) this->data[0].range(dnp::T_PTR+dnp::T_W-1, 0); // Keep the network portion unaffected
  
    this->data[1] = ((sc_uint<dnp::PHIT_W>) (rd_req.addr & ((1<<dnp::AP_W)-1)) << dnp::rresp::AP_PTR ) |
                    ((sc_uint<dnp::PHIT_W>) rd_req.len                         << dnp::rresp::LE_PTR ) |
                    ((sc_uint<dnp::PHIT_W>) rd_req.size                        << dnp::rresp::SZ_PTR) ;
  };
  
  template<typename T>
  inline void get_rd_req (T& rd_req, sc_uint<dnp::REORD_W>  &reord_tct) const {
    reord_tct    = (this->data[0] >> dnp::req::REORD_PTR) & ((1<<dnp::REORD_W)-1);
    rd_req.id    = (this->data[0] >> dnp::req::ID_PTR) & ((1<<dnp::ID_W)-1);
    rd_req.len   = (this->data[1] >> dnp::req::LE_PTR) & ((1<<dnp::LE_W)-1);
    rd_req.size  = (this->data[2] >> dnp::req::SZ_PTR) & ((1<<dnp::SZ_W)-1);
    rd_req.burst = (this->data[2] >> dnp::req::BU_PTR)  & ((1<<dnp::BU_W)-1);
    rd_req.addr  = ((((this->data[2]>>dnp::req::AH_PTR) & ((1<<dnp::AH_W)-1)) << dnp::AL_W) |
                     ((this->data[1]>>dnp::req::AL_PTR) & ((1<<dnp::AL_W)-1)));
  };
  
  template<typename T>
  inline void get_wr_req (const T& wr_req, sc_uint<dnp::REORD_W>  &reord_tct) const {this->get_rd_req(wr_req, reord_tct);};
  
  // Flit Constructors
  flit_dnp () {
    type = 0;
    vc   = 0;
    #pragma hls_unroll yes
    for(int i=0; i<PHIT_NUM; ++i)
      data[i] = 0;
  };

  flit_dnp(FLIT_TYPE _type, short int _vc, short int _src, short int _dst) {
    type    = _type;
    type    = _vc;
    data[0] = 0                 |
              (_src << dnp::S_PTR) |
              (_dst << dnp::D_PTR) ;
  };
  
  // Flit operators
	inline flit_dnp& operator = (const flit_dnp& rhs) {
		type   = rhs.type;
		vc     = rhs.vc;
		//dbg_id = rhs.dbg_id;
	  #pragma hls_unroll yes
    for(int i=0; i<PHIT_NUM; ++i) data[i] = rhs.data[i];

		return *this;
	};
  
  inline flit_dnp& operator = (const flit_dnp* rhs) {
		type   = rhs->type;
		vc     = rhs->vc;
    //dbg_id = rhs->dbg_id;
	  #pragma hls_unroll yes
    for(int i=0; i<PHIT_NUM; ++i) data[i] = rhs->data[i];

		return *this;
	};

	inline bool operator==(const flit_dnp& rhs) const {
    bool eq = (rhs.type == type) && (rhs.vc == vc);
    for(int i=0; i<PHIT_NUM; ++i) eq = eq && (data[i] == rhs.data[i]);
    return eq;
	}

	inline bool operator!=(const flit_dnp& rhs) const {
		return !(*this==rhs);
	}
  
  inline flit_dnp operator | (const flit_dnp& rhs) {
	  flit_dnp mule;
	  mule.type = type | rhs.type;
	  mule.vc   = vc   | rhs.vc;
    #pragma hls_unroll yes
    for(int i=0; i<PHIT_NUM; ++i) mule.data[i] = data[i] | rhs.data[i];
    
    return mule;
  };
  
  
  inline flit_dnp operator & (const flit_dnp& rhs) {
    flit_dnp mule;
    mule.type = type & rhs.type;
    mule.vc   = type & rhs.vc;
    #pragma hls_unroll yes
    for(int i=0; i<PHIT_NUM; ++i) mule.data[i] = data[i] & rhs.data[i];
    
    return mule;
  };
	
  
  inline flit_dnp and_mask(bool bit) const {
    flit_dnp mule;
    sc_uint<dnp::PHIT_W> mask = 0;
    
    #pragma hls_unroll yes
    for(int j=0; j<dnp::PHIT_W; ++j) mask[j] = mask[j] | (bit << j);
    
    mule.type = type & mask; //((mask<<1) | bit);
    mule.vc   = vc   & mask; //((mask<<1) | bit);
    #pragma hls_unroll yes
    for(int i=0; i<PHIT_NUM; ++i) mule.data[i] = data[i] & mask;
    
    return mule;
  };
	
//#ifndef __SYNTHESIS__
  // Non synthesizable functions. (printing, tracing)
  static std::string type_in_str(const flit_dnp& flit_in) {
	  if     (flit_in.type==HEAD)   return "H";
	  else if(flit_in.type==BODY)   return "B";
	  else if(flit_in.type==TAIL)   return "T";
	  else if(flit_in.type==SINGLE) return "S";
	  else                          return "X";
	}
 
	inline friend std::ostream& operator << ( std::ostream& os, const flit_dnp& flit_tmp ) {
	  //if (flit_tmp.type==HEAD || flit_tmp.type==SINGLE)
		  os << flit_tmp.get_vc() << flit_dnp::type_in_str(flit_tmp) <<"s" << flit_tmp.get_src() << "d" << flit_tmp.get_dst();// << " DBG_ID: 0x" << std::hex << flit_tmp.dbg_id;
	  //else
    //  os << flit_tmp.get_vc() << flit_dnp::type_in_str(flit_tmp) << "s? d?";
    
    os << " Pay: 0x";
    for(int i=PHIT_NUM-1; i>=0; --i) os << std::hex << flit_tmp.data[i].to_uint() << "_";
    
    #ifdef SYSTEMC_INCLUDED
    os << std::dec << "@" << sc_time_stamp();
		#else
    os << std::dec << "@" << "no-timed";
    #endif
    
		return os;
	}
//#endif

#ifdef SYSTEMC_INCLUDED
  // Only for SystemC
  inline friend void sc_trace(sc_trace_file* tf, const flit_dnp& flit, const std::string& name) {
		sc_trace(tf, flit.type, name + ".type");
		sc_trace(tf, flit.vc, name + ".vc");
		//sc_trace(tf, flit.dbg_id, name + ".dbg_id");
    for(int i=0; i<PHIT_NUM; ++i)
      sc_trace(tf, flit.data[i], name + ".data");
	}
#endif

  // Matchlib Marshaller requirement
  template<unsigned int Size>
  void Marshall(Marshaller<Size>& m) {
    //m& dbg_id;
    m& type;
    m& vc;
    #pragma hls_unroll yes
    for(int i=PHIT_NUM-1; i>=0; --i) m& data[i];
  };
  
};

#endif // __FLIT_DNP_H__

