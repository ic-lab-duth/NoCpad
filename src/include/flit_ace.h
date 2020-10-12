#ifndef __FLIT_DNP_H__
#define __FLIT_DNP_H__

#include "systemc.h"
#include "nvhls_connections.h"

#include "./dnp_ace_v0.h"

#ifndef __SYNTHESIS__
	#include <string>
	#include <iostream>
#endif

enum FLIT_TYPE {HEAD=0 , BODY=1, TAIL=3, SINGLE=2};

template<unsigned char PHIT_NUM>
struct flit_dnp {
  sc_uint<2>  type;
  //sc_uint<32> dbg_id;
  sc_uint<dnp::PHIT_W> data[PHIT_NUM];
  
  static const int width = 2+(PHIT_NUM*dnp::PHIT_W); // Matchlib Marshaller requirement
  
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
  inline sc_uint<dnp::V_W> get_vc()   const {return ((data[0] >> dnp::V_PTR) & ((1<<dnp::V_W)-1));};
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
  inline void set_vc(sc_uint<dnp::V_W>   vc  ) { data[0] = (data[0].range(dnp::PHIT_W-1, dnp::V_PTR+dnp::V_W) << (dnp::V_PTR+dnp::V_W)) |
                                                                (vc  << dnp::V_PTR) ;
                                                                //(data[0].range(dnp::V_PTR-1, 0));
  };
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
    data[0] = (data[0].range(dnp::PHIT_W-1, dnp::T_PTR+dnp::T_W) << (dnp::T_PTR+dnp::T_W)) |
              (type << dnp::T_PTR) |
              (qos  << dnp::Q_PTR) |
              (dst  << dnp::D_PTR) |
              (src  << dnp::S_PTR) |
              (vc   << dnp::V_PTR) ;
  };
  
  template<typename T>
  inline void set_rd_req (const T& rd_req) {
    this->data[0] = ((sc_uint<dnp::PHIT_W>) rd_req.snoop   << dnp::ace::req::SNP_PTR) |
                    ((sc_uint<dnp::PHIT_W>) rd_req.domain  << dnp::ace::req::DOM_PTR) |
                    ((sc_uint<dnp::PHIT_W>) rd_req.id      << dnp::ace::req::ID_PTR) |
                    (sc_uint<dnp::PHIT_W>) this->data[0].range(dnp::T_PTR+dnp::T_W-1, 0); // Keep the network portion unaffected
  
    this->data[1] = ((sc_uint<dnp::PHIT_W>) rd_req.len            << dnp::ace::req::LE_PTR) |
                    ((sc_uint<dnp::PHIT_W>)(rd_req.addr & 0xffff) << dnp::ace::req::AL_PTR) ;
  
    this->data[2] = ((sc_uint<dnp::PHIT_W>) rd_req.unique                  << dnp::ace::req::UNQ_PTR ) |
                    ((sc_uint<dnp::PHIT_W>) rd_req.barrier                 << dnp::ace::req::BAR_PTR ) |
                    ((sc_uint<dnp::PHIT_W>) rd_req.burst                   << dnp::ace::req::BU_PTR  ) |
                    ((sc_uint<dnp::PHIT_W>) rd_req.size                    << dnp::ace::req::SZ_PTR  ) |
                    ((sc_uint<dnp::PHIT_W>)(rd_req.addr >> dnp::ace::AL_W) << dnp::ace::req::AH_PTR) ;
  };
  
  template<typename T>
  inline void set_wr_req (const T& wr_req) {this->set_rd_req(wr_req);};
  
  template<typename T>
  inline void set_snoop_req (const T& snoop_req) {
    this->data[1] = ((sc_uint<dnp::PHIT_W>)snoop_req.snoop          << dnp::ace::creq::SNP_PTR) |
                   ((sc_uint<dnp::PHIT_W>)(snoop_req.addr & 0xffff) << dnp::ace::creq::AL_PTR) ;
    
    this->data[2] = ((sc_uint<dnp::PHIT_W>) snoop_req.prot                    << dnp::ace::creq::C_PROT_PTR ) |
                    ((sc_uint<dnp::PHIT_W>)(snoop_req.addr >> dnp::ace::AL_W) << dnp::ace::creq::AH_PTR) ;
  };
  
  template<typename T>
  inline void set_rd_resp (const T& rd_req) {
    this->data[0] = ((sc_uint<dnp::PHIT_W>) rd_req.burst   << dnp::ace::rresp::BU_PTR) |
                    ((sc_uint<dnp::PHIT_W>) rd_req.id      << dnp::ace::rresp::ID_PTR) |
                    (sc_uint<dnp::PHIT_W>) this->data[0].range(dnp::T_PTR+dnp::T_W-1, 0); // Keep the network portion unaffected
  
    this->data[1] = ((sc_uint<dnp::PHIT_W>) (rd_req.addr & ((1<<dnp::ace::AP_W)-1)) << dnp::ace::rresp::AP_PTR ) |
                    ((sc_uint<dnp::PHIT_W>) rd_req.len                              << dnp::ace::rresp::LE_PTR ) |
                    ((sc_uint<dnp::PHIT_W>) rd_req.size                             << dnp::ace::rresp::SZ_PTR) ;
  };
  
  template<typename T>
  inline void get_rd_req (T& rd_req) const {
    rd_req.id    = (this->data[0] >> dnp::ace::req::ID_PTR) & ((1<<dnp::ace::ID_W)-1);
    rd_req.len   = (this->data[1] >> dnp::ace::req::LE_PTR) & ((1<<dnp::ace::LE_W)-1);
    rd_req.size  = (this->data[2] >> dnp::ace::req::SZ_PTR) & ((1<<dnp::ace::SZ_W)-1);
    rd_req.burst = (this->data[2] >> dnp::ace::req::BU_PTR)  & ((1<<dnp::ace::BU_W)-1);
    rd_req.addr  = ((((this->data[2]>>dnp::ace::req::AH_PTR) & ((1<<dnp::ace::AH_W)-1)) << dnp::ace::AL_W) |
                     ((this->data[1]>>dnp::ace::req::AL_PTR) & ((1<<dnp::ace::AL_W)-1)));
    // rd_req.cache =
    // rd_req.auser =
    rd_req.snoop  = (this->data[0] >> dnp::ace::req::SNP_PTR) & ((1<<dnp::ace::SNP_W)-1);
    rd_req.domain = (this->data[0] >> dnp::ace::req::DOM_PTR) & ((1<<dnp::ace::DOM_W)-1);
  
    rd_req.barrier = (this->data[2] >> dnp::ace::req::BAR_PTR) & ((1<<dnp::ace::BAR_W)-1);
    //rd_req.unique =
  };
  
  template<typename T>
  inline void get_wr_req (const T& wr_req) const {this->get_rd_req(wr_req);};
  
  template<typename T>
  inline void get_snoop_req (T& snoop_req) const {
    snoop_req.snoop  = (this->data[1] >> dnp::ace::creq::SNP_PTR)    & ((1<<dnp::ace::SNP_W)-1);
    snoop_req.prot   = (this->data[2] >> dnp::ace::creq::C_PROT_PTR) & ((1<<dnp::ace::C_PROT_W)-1);
    snoop_req.addr   = ((((this->data[2]>>dnp::ace::creq::AH_PTR) & ((1<<dnp::ace::AH_W)-1)) << dnp::ace::AL_W) |
                        ((this->data[1]>>dnp::ace::creq::AL_PTR)  & ((1<<dnp::ace::AL_W)-1)));
  };
  
  // Flit Constructors
  flit_dnp () {
    type = 0;
    #pragma hls_unroll yes
    for(int i=0; i<PHIT_NUM; ++i)
      data[i] = 0;
  };

  flit_dnp(FLIT_TYPE _type, short int _src, short int _dst) {
    type    = _type;
    data[0] = 0                 |
              (_src << dnp::S_PTR) |
              (_dst << dnp::D_PTR) ;
  };
  
  // Flit operators
	inline flit_dnp& operator = (const flit_dnp& rhs) {
		type   = rhs.type;
		//dbg_id = rhs.dbg_id;
	  #pragma hls_unroll yes
    for(int i=0; i<PHIT_NUM; ++i) data[i] = rhs.data[i];

		return *this;
	};
  
  inline flit_dnp& operator = (const flit_dnp* rhs) {
		type   = rhs->type;
    //dbg_id = rhs->dbg_id;
	  #pragma hls_unroll yes
    for(int i=0; i<PHIT_NUM; ++i) data[i] = rhs->data[i];

		return *this;
	};

	inline bool operator==(const flit_dnp& rhs) const {
    bool eq = (rhs.type == type);
    for(int i=0; i<PHIT_NUM; ++i) eq = eq && (data[i] == rhs.data[i]);
    return eq;
	}

	inline bool operator!=(const flit_dnp& rhs) const {
		return !(*this==rhs);
	}
  
  inline flit_dnp operator | (const flit_dnp& rhs) {
	  flit_dnp mule;
	  mule.type = type | rhs.type;
    #pragma hls_unroll yes
    for(int i=0; i<PHIT_NUM; ++i) mule.data[i] = data[i] | rhs.data[i];
    
    return mule;
  };
  
  
  inline flit_dnp operator & (const flit_dnp& rhs) {
    flit_dnp mule;
    mule.type = type & rhs.type;
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
		  os << flit_tmp.get_vc() << flit_dnp::type_in_str(flit_tmp) <<"s" << flit_tmp.get_src() << "d" << flit_tmp.get_dst() << "v" << flit_tmp.get_vc();// << " DBG_ID: 0x" << std::hex << flit_tmp.dbg_id;
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
    #pragma hls_unroll yes
    //for(int i=0; i<PHIT_NUM; ++i) m& data[i];
    for(int i=PHIT_NUM-1; i>=0; --i) m& data[i];
    //m& src;
    //m& dst;
  };
  
};


struct flit_ack {
    sc_uint<2>        type;
    sc_uint<dnp::S_W> src;
    sc_uint<dnp::D_W> dst;
    sc_uint<1>        rack;
    sc_uint<1>        wack;
    
    static const int width = 2+dnp::S_W+dnp::D_W+1+1; // Matchlib Marshaller requirement
    
    flit_ack(unsigned type_=0, unsigned src_=0, unsigned dst_=0, bool rack_=0, bool wack_=0) :
            type(type_), src(src_), dst(dst_), rack(rack_), wack(wack_)
    {};
    
    // helping functions to retrieve flit info (e.g. flit type, source, destination)
    inline bool performs_rc()   { return ((type == HEAD) || (type == SINGLE)); }
    inline bool resets_states() { return (type == TAIL); }
    
    inline bool is_head()   {return (type==HEAD);};
    inline bool is_tail()   {return (type==TAIL);};
    inline bool is_body()   {return (type==BODY);};
    inline bool is_single() {return (type==SINGLE);};
    
    inline sc_uint<dnp::D_W> get_dst()  const {return dst;};
    inline sc_uint<dnp::S_W> get_src()  const {return src;};
    inline sc_uint<dnp::T_W> get_type()  const {return 0;};
    
    inline bool is_rack()  const {return rack;};
    inline bool is_wack()  const {return wack;};
    
    
    // Non synthesizable functions. (printing, tracing)
    static std::string type_in_str(const flit_ack& flit_in) {
      if     (flit_in.type==HEAD)   return "H";
      else if(flit_in.type==BODY)   return "B";
      else if(flit_in.type==TAIL)   return "T";
      else if(flit_in.type==SINGLE) return "S";
      else                          return "X";
    }
    
    inline friend std::ostream& operator << ( std::ostream& os, const flit_ack& flit_tmp ) {
      os << flit_ack::type_in_str(flit_tmp) <<"s" << flit_tmp.get_src() << "d" << flit_tmp.get_dst() << " rack: " << flit_tmp.is_rack() << " wack: " << flit_tmp.is_wack();
      #ifdef SYSTEMC_INCLUDED
        os << std::dec << "@" << sc_time_stamp();
      #else
        os << std::dec << "@" << "no-timed";
      #endif
      
      return os;
    }

    #ifdef SYSTEMC_INCLUDED
    // Only for SystemC
    inline friend void sc_trace(sc_trace_file* tf, const flit_ack& flit, const std::string& name) {
      sc_trace(tf, flit.type, name + ".type");
      sc_trace(tf, flit.src, name + ".src");
      sc_trace(tf, flit.dst, name + ".dst");
      
      sc_trace(tf, flit.rack, name + ".rack");
      sc_trace(tf, flit.wack, name + ".wack");
    }
    #endif
    
    // Matchlib Marshaller requirement
    template<unsigned int Size>
    void Marshall(Marshaller<Size>& m) {
      m& type;
      m& src;
      m& dst;
      m& rack;
      m& wack;
    };
};




#endif // __FLIT_DNP_H__

