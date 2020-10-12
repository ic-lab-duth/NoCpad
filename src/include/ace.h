#ifndef _AXI_ACE_H_
#define _AXI_ACE_H_

#include <systemc>
#include <nvhls_connections.h>
#include <nvhls_assert.h>
#include <nvhls_message.h>
#include <nvhls_module.h>

#include <UIntOrEmpty.h>

#include <axi/axi4_encoding.h>
#include <axi/axi4_configs.h>
#include "./axi_for_ace.h"

/**
 * \brief The axi namespace contains classes and definitions related to the ACE standard.
 * \ingroup ACE
 */
namespace ace {

/**
 * \brief The base ACE class extending AXI with cache coherence.
 * \ingroup ACE
 *
 * \tparam Cfg        A valid AXI config.
 *
 * \par Overview
 * axi4 defines the AXI base class.  Based on the provided Cfg, the bitwidths of
 * the various fields of the AXI specification are defined, and classes and
 * convenience functions can be used to instantiate AXI Connections, wire them
 * together, and use them to implement the AXI protocol.
 * - Each AXI signal is defined as a UIntOrEmpty of an appropriate width, allowing
 * for the presence of 0-width fields when they can be elided entirely.
 * - If useWriteResponses = 0, the B channel is not removed entirely (for
 * implementation convenience), but is reduced to minimum width.
 * - All AW and AR fields are identical, and are combined into a common
 * AddrPayload type.
 *
 */

template <typename Cfg>
class ace5 : public ace::axi4<Cfg> {
public:

  enum {
    C_ADDR_WIDTH  = Cfg::addrWidth,
    C_SNOOP_WIDTH = 4,
    C_PROT_WIDTH  = 3,
    C_RESP_WIDTH  = 5,
    C_CACHE_WIDTH = Cfg::CacheLineWidth,
    C_DATA_CHAN_WIDTH = Cfg::CacheLineWidth,
  };
  
  typedef typename ace::axi4<Cfg>::Addr Addr;
  
    /**
   * \brief A struct for ACE Snoop Address channel fields
   */
  struct AC : public nvhls_message {
    typedef NVUINTW(C_SNOOP_WIDTH) Snoop;
    typedef typename nvhls::UIntOrEmpty<C_PROT_WIDTH>::T Prot;
    
    Addr  addr;
    Snoop snoop;
    Prot  prot;
    
    static const unsigned int width = C_ADDR_WIDTH + C_SNOOP_WIDTH + C_PROT_WIDTH;

    AC () {
      addr = 0;
      snoop  = 0;
      if(C_PROT_WIDTH > 0) prot = 0;
    }

    template <unsigned int Size>
    void Marshall(Marshaller<Size> &m) {
      m &addr;
      m &snoop;
      m &prot;
    }

    #ifdef CONNECTIONS_SIM_ONLY
      inline friend void sc_trace(sc_trace_file *tf, const AC& v, const std::string& NAME ) {
        sc_trace(tf,v.addr,   NAME + ".addr");
        sc_trace(tf,v.snoop,  NAME + ".snoop");
        if(C_PROT_WIDTH > 0)
          sc_trace(tf,v.prot, NAME + ".prot");
      }

      inline friend std::ostream& operator<<(ostream& os, const AC& rhs)
      {
        #ifdef LOG_MSG_WIDTHS
          os << std::dec;
          os << "addr:" << rhs.addr.width << " ";
          os << "snoop:" << rhs.snoop.width << " ";
          if(C_PROT_WIDTH > 0)
            os << "prot:" << rhs.prot.width << " ";
        #else
          os << std::hex;
          os << "Addr:" << rhs.addr << " ";
          os << "Snp:" << rhs.snoop << " ";
        if(C_PROT_WIDTH > 0)
          os << "Prot:" << rhs.prot << " ";
        os << std::dec;
        #endif
          return os;
      }
    #endif
  };
  
  
  /**
   * \brief A struct for ACE Snoop Response channel fields
  */
  struct CR : public nvhls_message {
    typedef NVUINTW(C_RESP_WIDTH) Resp;
    
    Resp resp;
    
    static const unsigned int width = C_RESP_WIDTH;

    CR () {
      resp = 0;
    }

    template <unsigned int Size>
    void Marshall(Marshaller<Size> &m) {
      m &resp;
    }

    #ifdef CONNECTIONS_SIM_ONLY
      inline friend void sc_trace(sc_trace_file *tf, const CR& v, const std::string& NAME ) {
        sc_trace(tf,v.resp,    NAME + ".resp");
      }

      inline friend std::ostream& operator<<(ostream& os, const CR& rhs)
      {
        #ifdef LOG_MSG_WIDTHS
          os << std::dec;
          os << "Resp:" << rhs.resp.width << " ";
        #else
          os << std::hex;
          os << "Resp:" << rhs.resp << " ";
          os << std::dec;
        #endif
          return os;
      }
    #endif
  };
  
  
  /**
   * \brief A struct for ACE Snoop Data channel fields
  */
  struct CD : public nvhls_message {
    typedef NVUINTW(C_DATA_CHAN_WIDTH) Data;
    typedef NVUINTW(1)            Last;
    
    Data data;
    Last last;
    
    static const unsigned int width = C_DATA_CHAN_WIDTH + 1;

    CD () {
      data = 0;
      last = 0;
    }

    template <unsigned int Size>
    void Marshall(Marshaller<Size> &m) {
      m &data;
      m &last;
    }

    #ifdef CONNECTIONS_SIM_ONLY
      inline friend void sc_trace(sc_trace_file *tf, const CD& v, const std::string& NAME ) {
        sc_trace(tf,v.data,    NAME + ".data");
        sc_trace(tf,v.last,    NAME + ".last");
      }

      inline friend std::ostream& operator<<(ostream& os, const CD& rhs)
      {
        #ifdef LOG_MSG_WIDTHS
          os << std::dec;
          os << "data:" << rhs.data.width << " ";
          os << "last:" << rhs.last.width << " ";
        #else
          os << std::hex;
          os << "Data:" << rhs.data << " ";
          os << "last:" << rhs.last << " ";
          os << std::dec;
        #endif
          return os;
      }
    #endif
  };
  
  /**
   * \brief A struct for ACE Snoop ACK channels
  */
  struct RACK : public nvhls_message {
    typedef NVUINTW(1) Rack;
    Rack rack;
    
    static const unsigned int width = 1;
    
      RACK () {
      rack = 0;
    }

    template <unsigned int Size>
    void Marshall(Marshaller<Size> &m) {
      m &rack;
    }

    #ifdef CONNECTIONS_SIM_ONLY
      inline friend void sc_trace(sc_trace_file *tf, const RACK& v, const std::string& NAME ) {
        sc_trace(tf,v.rack,    NAME + ".rack");
      }

      inline friend std::ostream& operator<<(ostream& os, const RACK& rhs)
      {
        #ifdef LOG_MSG_WIDTHS
          os << std::dec;
          os << "rack:" << rhs.rack.width << " ";
        #else
          os << std::dec;
          os << "Rack:" << rhs.rack << " ";
        #endif
          return os;
      }
    #endif
  };
    
  struct WACK : public nvhls_message {
      typedef NVUINTW(1) Wack;
      Wack wack;
      
      static const unsigned int width = 1;
    
      WACK () {
        wack = 0;
      }
      
      template <unsigned int Size>
      void Marshall(Marshaller<Size> &m) {
        m &wack;
      }

    #ifdef CONNECTIONS_SIM_ONLY
      inline friend void sc_trace(sc_trace_file *tf, const WACK& v, const std::string& NAME ) {
        sc_trace(tf,v.wack,    NAME + ".wack");
      }
      
      inline friend std::ostream& operator<<(ostream& os, const WACK& rhs)
      {
      #ifdef LOG_MSG_WIDTHS
        os << std::dec;
        os << "wack:" << rhs.wack.width << " ";
      #else
        os << std::dec;
        os << "Wack:" << rhs.wack << " ";
      #endif
        return os;
      }
    #endif
  };
  
  
  
  
  /**
   * \brief The ACE extension channels.
   *
   * Each Connections implementation contains two ready-valid interfaces,
   * AC for snoop address, CR for snoop response, CD for snoop data, ACK for acknowledgment
   */
  class cache {
   public:
    /**
     * \brief The ACE cache channel, used for connecting Caching components.
     */
    template <Connections::connections_port_t PortType = AUTO_PORT>
    class chan {
     public:
     typedef Connections::Combinational<AC, PortType>  ACChan;
     typedef Connections::Combinational<CR, PortType>  CRhan;
     typedef Connections::Combinational<CD, PortType>  CDChan;
     //typedef Connections::Combinational<ACK, PortType> ACKChan;

      ACChan  ac;  // IC to master (DIR to IC)
      CRhan   cr;  // master to IC (IC to DIR)
      CDChan  cd;  // master to IC (IC to DIR)
      //ACKChan ack; // Master to IC (IC to DIR???)

      chan(const char *name)
        :
        ac(nvhls_concat(name, "_ac")),
        cr(nvhls_concat(name, "_cr")),
        cd(nvhls_concat(name, "_cd"))
        //ack(nvhls_concat(name, "_ack"))
      {};

      // TODO: Implement AXI protocol checker
    }; // read::chan

    /**
     * \brief The ACE cache master/IC port.  This port has AC Snoop request as input and CR-CD-ACK channels as output.
     */
    template <Connections::connections_port_t PortType = AUTO_PORT>
    class master {
     public:
      typedef Connections::In<AC, PortType>   ACPort;
      typedef Connections::Out<CR, PortType>  CRPort;
      typedef Connections::Out<CD, PortType>  CDPort;
      //typedef Connections::Out<ACK, PortType> ACKPort;

      ACPort  ac;
      CRPort  cr;
      CDPort  cd;
      //ACKPort ack;

      master(const char *name)
        :
        ac(nvhls_concat(name, "_ac")),
        cr(nvhls_concat(name, "_cr")),
        cd(nvhls_concat(name, "_cd"))
        //ack(nvhls_concat(name, "_ack"))
      {}

      void reset() {
        ac.Reset();
        cr.Reset();
        cd.Reset();
        //ack.Reset();
      }
      
      AC   snp_rcv()            { return ac.Pop(); }
      bool snp_rcv_nb(AC &addr) { return ac.PopNB(addr); }
      
      void snp_resp(const CR &resp)    { cr.Push(resp); }
      bool snp_resp_nb(const CR &resp) { return cr.PushNB(resp); }
      
      void snp_data(const CD &data)    { cd.Push(data); }
      bool snp_data_nb(const CD &data) { return cd.PushNB(data); }
      
      //void snp_ack(const ACK &ack_r)    { ack.Push(ack_r); }
      //bool snp_ack_nb(const ACK &ack_r) { return ack.PushNB(ack_r); }

      template <class C>
      void operator()(C &c) {
        ac(c.ac);
        cr(c.cr);
        cd(c.cd);
        //ack(c.ack);
      }
    }; // cache::master
    
    /**
     * \brief The ACE cache master/IC port.  This port has AC Snoop request as output and CR-CD-ACK channels as input.
     */
    template <Connections::connections_port_t PortType = AUTO_PORT>
    class dir {
     public:
      typedef Connections::Out<AC, PortType>  ACPort;
      typedef Connections::In<CR, PortType>   CRPort;
      typedef Connections::In<CD, PortType>   CDPort;
      //typedef Connections::In<ACK, PortType>  ACKPort;

      ACPort  ac;
      CRPort  cr;
      CDPort  cd;
      //ACKPort ack;

      dir(const char *name)
        :
        ac(nvhls_concat(name, "_ac")),
        cr(nvhls_concat(name, "_cr")),
        cd(nvhls_concat(name, "_cd"))
        //ack(nvhls_concat(name, "_ack"))
      {}

      void reset() {
        ac.Reset();
        cr.Reset();
        cd.Reset();
        //ack.Reset();
      }
      
      void snp_snd(AC &addr)    { ac.Push(addr); }
      bool snp_snd_nb(AC &addr) { return ac.PushNB(addr); }
      
      CR   snp_resp_rcv()                  { return cr.Pop(); }
      bool snp_resp_rcv_nb(const CR &resp) { return cr.PopNB(resp); }
      
      CD snp_data_rcv( )             { return cd.Pop(); }
      bool snp_data_rcv_nb(CD &data) { return cd.PopNB(data); }
      
      //ACK snp_ack_rcv( )              { return ack.Pop(); }
      //bool snp_ack_rcv_nb(ACK &ack_r) { return ack.PopNB(ack_r); }

      template <class C>
      void operator()(C &c) {
        ac(c.ac);
        cr(c.cr);
        cd(c.cd);
        //ack(c.ack);
      }
    }; // cache::dir
  }; // cache
}; // ace5

 /**
 * \brief Hardcoded values associated with the ACE standard.
 * \ingroup ACE
 *
 * \par
 * These enumerated values are defined by the ACE standard and should not be modified.
 *
 */
class ACE_Encoding : public axi::AXI4_Encoding {
  public:
    /**
    * \brief Hardcoded values for the AxDOMAIN field.
    */
    class AxDOMAIN {
    public:
      enum {
        _WIDTH = 2,  // bits
        
        NON_SHARE    = 0,
        INNER_SHARE  = 1,
        OUTER_SHARE  = 2,
        SYSTEM_SHARE = 3,
      };
    }; // Domain
    
    class ARSNOOP {
    public:
      enum {
        _WIDTH = 4,  // bits
        // Np-Snoop
        
        // Coherent
        RD_ONCE    = 0x0, // Or Read NoSnoop if Domain is Non-Shared or System
        RD_SHARED  = 0x1,
        RD_CLEAN   = 0x2,
        RD_NOT_SHARED_DIRTY = 0x3,
        RD_UNIQUE    = 0x7,
        CLEAN_UNIQUE = 0xB,
        MAKE_UNIQUE  = 0xC,
        // Cache maintenance
        CLEAN_SHARED  = 0x8,
        CLEAN_INVALID = 0x9,
        MAKE_INVALID  = 0xD,
      };
    }; // ARSNOOP
    
    class AWSNOOP {
    public:
      enum {
        _WIDTH = 3,  // bits
        // Np-Snoop
        // Coherent
        WR_UNIQUE = 0, // Or Write NoSnoop id DOMAIN is Not-Shared or System
        WR_LINE_UNIQUE  = 1,
        // Memory update
        WR_CLEAN = 2,
        WR_BACK  = 3, // No-Snoop
        EVICT    = 4, // No-Snoop
        WR_EVICT = 5, // No-Snoop
      };
    }; // AWSNOOP
    
    class ACSNOOP {
    public:
      enum {
        _WIDTH = 4,  // bits
        
        RD_ONCE   = 0x0,
        RD_SHARED = 0x1,
        RD_CLEAN  = 0x2,
        RD_NOT_SHARED_DIRTY = 0x3,
        RD_UNIQUE     = 0x7,
        CLEAN_SHARED  = 0x8,
        CLEAN_INVALID = 0x9,
        MAKE_INVALID  = 0xD,
        DVM_COMPLETE  = 0xE,
        DVM_MESSAGE   = 0xF,
      };
    }; // AWSNOOP
  }; // End of ACE_Encoding
    
  inline NVUINTW(ACE_Encoding::ACSNOOP::_WIDTH) rd_2_snoop (NVUINTW(ACE_Encoding::ARSNOOP::_WIDTH) &request_in) {
    if      (request_in == ACE_Encoding::ARSNOOP::RD_ONCE) // 0 -> 0
      return ACE_Encoding::ACSNOOP::RD_ONCE;
    else if (request_in == ACE_Encoding::ARSNOOP::RD_CLEAN) // 2 -> 2
      return ACE_Encoding::ACSNOOP::RD_CLEAN;
    else if (request_in == ACE_Encoding::ARSNOOP::RD_NOT_SHARED_DIRTY) // 3 -> 3
      return ACE_Encoding::ACSNOOP::RD_NOT_SHARED_DIRTY;
    else if (request_in == ACE_Encoding::ARSNOOP::RD_SHARED) // 1-> 1
      return ACE_Encoding::ACSNOOP::RD_SHARED;
    else if (request_in == ACE_Encoding::ARSNOOP::RD_UNIQUE) // 7 -> 7
      return ACE_Encoding::ACSNOOP::RD_UNIQUE;
    else if ((request_in == ACE_Encoding::ARSNOOP::CLEAN_UNIQUE)  ||
             (request_in == ACE_Encoding::ARSNOOP::CLEAN_INVALID)   ) // 11, 9 -> 9
      return ACE_Encoding::ACSNOOP::CLEAN_INVALID;
    else if ((request_in == ACE_Encoding::ARSNOOP::MAKE_UNIQUE)  ||
             (request_in == ACE_Encoding::ARSNOOP::MAKE_INVALID)    ) // 12, 13 -> 13
      return ACE_Encoding::ACSNOOP::MAKE_INVALID;
    else if (request_in == ACE_Encoding::ARSNOOP::CLEAN_SHARED) // 8 -> 8
      return ACE_Encoding::ACSNOOP::CLEAN_SHARED;
    else {
      NVHLS_ASSERT_MSG(0, "Read coherent access is out of valid snoop values.")
    }
    return 0;
  }; // End of rd_2_snoop
    
  inline NVUINTW(ACE_Encoding::ACSNOOP::_WIDTH) wr_2_snoop (NVUINTW(ACE_Encoding::ARSNOOP::_WIDTH) &request_in) {
    if (request_in == ACE_Encoding::AWSNOOP::WR_UNIQUE) // 0 -> 9
      return ACE_Encoding::ACSNOOP::CLEAN_INVALID;
    else if (request_in == ACE_Encoding::AWSNOOP::WR_LINE_UNIQUE) // 1 -> 13
      return ACE_Encoding::ACSNOOP::MAKE_INVALID;
    else {
      NVHLS_ASSERT_MSG(0, "Coherent access does not match any expected at Home.")
    }
    return 0;
  }; // End of wr_2_snoop
}; // ace


#endif // _AXI_ACE_H_
