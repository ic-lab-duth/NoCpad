#ifndef __ARBITERS_HEADER__
#define __ARBITERS_HEADER__

enum arb_type {FIXED, MATRIX, ROUND_ROBIN, WEIGHTED_RR, DEFICIT_RR, STRATIFIED_RR, PHASE};


template<unsigned SIZE, arb_type ARB_TYPE, unsigned S=0, unsigned DOMAINS=0>
class arbiter {

public:
  arbiter();
  unsigned arbitrate();
};


/* FUNCTION: Fixed Priority Arbiter
 * INPUT:    Array of bools
 * OUTPUT:   Unsigned integer pointer to bit position
 * -----------------------------------------
 *
 */
template<unsigned SIZE>
class arbiter<SIZE, FIXED, 0, 0> {
private:

public:
  arbiter(){
  }
  
  unsigned arbitrate( bool inp[SIZE] ) {
    unsigned grants;
    bool     found = false;

#pragma hls_unroll yes
    for (int i=0; i<SIZE; i++) {
      if (inp[i] && !found) {
        grants = i;
        found = true;
      }
    }
    
    return grants;
  };
};

/* FUNCTION: Round Robin Arbiter
 * INPUT:    Array of bools
 * OUTPUT:   Unsigned integer pointer to bit position
 * -----------------------------------------
 *
 */
template<unsigned SIZE>
class arbiter<SIZE, ROUND_ROBIN, 0, 0> {
private:
  unsigned priority;
  sc_uint<SIZE> priority_therm;

public:
  arbiter() {
    priority = 0;
    priority_therm = 0;
  }
  
  //#pragma hls_design ccore
  //#pragma hls_ccore_type combinational
  unsigned arbitrate(bool inp[SIZE]) {
    
    bool found_hp = false;
    bool found_lp = false;
    
    unsigned grant_hp = 0;
    unsigned grant_lp = 0;
    unsigned grants = 0;

#pragma hls_unroll yes
    for (int i = 0; i < SIZE; i++) {
      // split arbitration to keep for-loop bounds constant - HLS requirement
      // if requests belong to the high priority segment
      if (i >= priority) {
        if (inp[i] && !found_hp) {
          grant_hp = i;
          found_hp = true;
        }
      } else { // requests that belong to low priority
        if (inp[i] && !found_lp) {
          grant_lp = i;
          found_lp = true;
        }
      }
    }
    
    grants = (found_hp) ? grant_hp : grant_lp;
    if (found_hp || found_lp) {
      priority = ((grants + 1) == SIZE) ? 0 : (grants + 1);
    }
    
    return grants;
  };

  
  //#pragma hls_design ccore
  //#pragma hls_ccore_type combinational
  //#pragma hls_map_to_operator le_arbiter
  bool arbitrate(const sc_uint<SIZE> reqs_i, sc_uint<SIZE>&  grants_o) {
    sc_uint<SIZE> req_lp = reqs_i & (~priority_therm);
    sc_uint<SIZE> req_hp = reqs_i & priority_therm;
  
    sc_uint<SIZE> grants_lp = ((~req_lp) + 1) & req_lp;
    sc_uint<SIZE> grants_hp = ((~req_hp) + 1) & req_hp;
  
    bool anygrant = reqs_i.or_reduce();
    grants_o = grants_hp.or_reduce() ? grants_hp : grants_lp;
    
    // OH to THERM
    if (anygrant) priority_therm = ~((grants_o << 1) - 1);
  
    return anygrant;
  };
  
  
};

/* FUNCTION: Matrix Arbiter
 * INPUT:    Array of bools
 * OUTPUT:   Unsigned integer pointer to bit position
 * -----------------------------------------
 *
 */
template<unsigned SIZE>
class arbiter<SIZE, MATRIX, 0, 0> {
private:
  bool mat[SIZE][SIZE];
  bool mat_v2[SIZE][SIZE];
  //sc_uint<SIZE> matrix_reg[SIZE]; // a word reflects a vertical line of the matrix

public:
  
  arbiter(){

#pragma hls_unroll yes
    for (int i=0; i<SIZE; i++) {

#pragma hls_unroll yes
      for (int j=0; j<SIZE; j++) {
        mat_v2[i][j] = (i>j);

        if (i < j) {
          mat[i][j] = true;
        }
      }
    }
  }
  
  unsigned arbitrate( bool inp[SIZE] ) {
    
    bool found = false;
    
    unsigned grants = 0;

#pragma hls_unroll yes
    for (int i=0; i<SIZE; i++) {
      if (inp[i] && !found) {
        found = true;

#pragma hls_unroll yes
        for (int j=0; j<SIZE; j++) {
          if ( (i < j) && (!mat[i][j] && inp[j])) {
            found = false;
          } else {
            if ( (i > j) && (mat[i][j] && inp[j])) {
              found = false;
            }
          }
        }
        if (found) {
          grants = i;
        }
      }
    }
    
    if (found) {
      // If a grants is given - change matrix values
#pragma hls_unroll yes
      for (int i=0; i<SIZE; i++) {
        if (i > grants) {
          mat[grants][i] = false;
        } else {
          if (i < grants) {
            mat[i][grants] = true;
          }
        }
      }
    }
    
    return grants;
  };
  
  bool arbitrate(const sc_uint<SIZE> reqs_i, sc_uint<SIZE>&  grants_o) {
    #pragma hls_unroll yes
    for (int i=0; i<SIZE; ++i) { // Horizontal
      sc_uint<SIZE> vert_line;
      #pragma hls_unroll yes
      for (int j=0; j<SIZE; ++j) { // Vertical
        if (i==j) vert_line[j] = false;
        else      vert_line[j] = reqs_i[j] && mat_v2[i][j];
      }
      grants_o[i] = !(vert_line.or_reduce()) && reqs_i[i];
    }
    //NVHLS_ASSERT_MSG(dbg_grants<=1, "More than one granted!")

    // Update table
    #pragma hls_unroll yes
    for (int i=0; i<SIZE; ++i) { // Horizontal
      #pragma hls_unroll yes
      for (int j=0; j<SIZE; ++j) { // Vertical
        if (i!=j) {
          if      (grants_o[i]) mat_v2[i][j] = true;
          else if (grants_o[j]) mat_v2[i][j] = false;
        }
      }
    }
    
    bool anygrant = reqs_i.or_reduce();
    return anygrant;
  };
  
};

/* FUNCTION: Wighted Round Robin Arbiter
 * INPUT:    Array of bools
 * OUTPUT:   Unsigned integer pointer to bit position
 * -----------------------------------------
 *
 */
template<unsigned SIZE>
class arbiter<SIZE, WEIGHTED_RR, 0, 0> {
private:
  bool     Weights[SIZE][SIZE];
  bool     unvisited[SIZE];
  unsigned scanCycle;
  
  /* FUNCTION: Priority Enforcer
   * INPUT:    Integer Value
   * OUTPUT:   Pointer to the least significant
   *           non zerob bit.
   * -----------------------------------------
   *
   */
  unsigned priorityEnforcer( unsigned inp ) {
    
    unsigned pbit=0;
    bool found = false;

#pragma hls_unroll yes
    for (int i=0; i<SIZE; i++) {
      if ( ((inp & 1<<i) >0) && !found ) {
        found = true;
        pbit = i;
      }
    }
    
    return pbit;
  }

public:
  
  arbiter(){
    
    scanCycle = 1;
#pragma hls_unroll yes
    for (int i=0; i<SIZE; i++) {
      
      unvisited[i] = true;
#pragma hls_unroll yes
      for (int j=0; j<SIZE; j++) {
        
        if (j == i)
          Weights[i][j] = true;
        else
          Weights[i][j] = false;
      }
    }
  }
  
  unsigned arbitrate( bool inp[SIZE] ) {
    
    bool sterCyles[SIZE];
    bool isSterile;
    bool found     = false;
    
    unsigned pbit;
    unsigned sterile = 0;;
    unsigned grants  = 0;

#pragma hls_unroll yes
    for (int i=0; i<SIZE; i++) {
      sterCyles[i] = false;
    }

#pragma hls_unroll yes
    for (int k=0;k<SIZE; k++) { // repeat until a non sterile scan cycle
      
      pbit = priorityEnforcer(scanCycle);

#pragma hls_unroll yes
      for (int i=0; i<SIZE; i++) { // check each input flow
        
        if (!found && inp[i] && unvisited[i] && Weights[i][pbit]) {
          unvisited[i] = false;
          found = true;
          grants = i;
        }
      }
      
      if (!found) { // if no grant was given
        
        isSterile = true;

#pragma hls_unroll yes
        for (int i=0; i<SIZE; i++) { // update unvisited flows
          
          if ( !unvisited[i] ) {
            isSterile = false;
          }
          unvisited[i] = true;
        }
        
        if (isSterile) { // update scan cycle avoiding sterile cycles
          
          sterCyles[pbit] = true;

#pragma hls_unroll yes
          for (int i=0; i<SIZE; i++) {
            if (sterCyles[i] && ((scanCycle | 1<<i)!=scanCycle) ) {
              sterile = sterile + (1<<i);
            }
          }
        }
        
        scanCycle = scanCycle + sterile + 1;
        if (scanCycle >= ((1<<SIZE)-1) ) {
          scanCycle = sterile + 1;
        }
      }
    }
    
    return grants;
  };
  
  
  /* FUNCTION: WRR Arbiter :: setWeights
   * INPUT:    Array of unsigned weight values
   * OUTPUT:
   * -----------------------------------------
   * Input weight for each flow is given as a
   * percentage of the bandwidth is asks for.
   * For example if the value of the weight is 75,
   * this flow asks for the 75% of the bandwidth.
   */
  void setWeights( unsigned inp[SIZE] ) {
    
    unsigned tmp;

#pragma hls_unroll yes
    for (int i=0; i<SIZE; i++) {
      
      tmp = inp[i];

#pragma hls_unroll yes
      for (int j=0; j<SIZE; j++) {
        
        if (tmp >= 100/(1<<(j+1))) {
          Weights[i][j] = true;
          tmp = tmp - 100/(1<<(j+1));
        } else {
          Weights[i][j] = false;
        }
      }
    }
  }
};


/* FUNCTION: Deficit Round Robin Arbiter
 * INPUT:    Array of bools
 * OUTPUT:   Unsigned integer pointer to bit position
 * -----------------------------------------
 *
 */
template<unsigned SIZE>
class arbiter<SIZE, DEFICIT_RR, 0, 0> {
private:
  unsigned priority;
  
  struct flowQueue {
    unsigned packetSize[5];
    unsigned maxIndex;
  };
  unsigned  D[SIZE];
  unsigned  Q[SIZE];
  bool      ActiveList[SIZE];
  flowQueue F[SIZE];

public:
  arbiter(){

#pragma hls_unroll yes
    for (int i=0; i<SIZE; i++) {
      ActiveList[i] = false;
      F[i].maxIndex = 0;
      D[i] = 0;
      Q[i] = 30; // maybe a function to set quantum of BW per input
    }
  }
  
  unsigned arbitrate( bool inp[SIZE], unsigned inp_size[SIZE] ) {
    
    bool found_hp = false;
    bool found_lp = false;
    
    unsigned grant_hp = 0;
    unsigned grant_lp = 0;
    unsigned grants   = 0;



#pragma hls_unroll yes
    for (int i=0; i<SIZE; i++) {
      
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      if ( inp[i] && F[i].maxIndex<5) {
        ActiveList[i] = true;
        F[i].packetSize[F[i].maxIndex] = inp_size[i];
        F[i].maxIndex++;
      }
      
      D[i] = ( ActiveList[i] ) ? (D[i] + Q[i]) : 0;
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      
      if (i >= priority) {
        if (F[i].packetSize[0]<=D[i] && ActiveList[i] && !found_hp) {
          
          grant_hp = i;
          found_hp = true;
        }
      } else {
        if (F[i].packetSize[0]<=D[i] && ActiveList[i] && !found_lp) {
          
          grant_lp = i;
          found_lp = true;
        }
      }
    }
    
    grants = (found_hp) ? grant_hp: grant_lp;
    
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    // Update Queue and Deficit Counter for flow i
    D[grants] = D[grants] - F[grants].packetSize[0];
#pragma hls_unroll yes
    for (int i=0; i<4; i++) {
      F[grants].packetSize[i] = F[grants].packetSize[i+1];
    }
    F[grants].maxIndex--;
    
    if ( F[grants].maxIndex == 0 ) {
      ActiveList[grants] = false;
      D[grants] = 0;
    }
    
    if (found_hp || found_lp && (D[grants]<F[grants].packetSize[0])) {
      priority = ((grants + 1) == SIZE) ? 0 : (grants+1);
    }
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    return grants;
  };
};

#endif // __ARBITERS_HEADER__
