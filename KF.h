
#ifndef KF_h
#define KF_h
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class KF {
  public:
    //constructor
    KF();
    //methods
    void getABC(float  A[4], float B[2], float C[2]);
    void getQR(float Q[4], float R);
    void kalman(float x[2], float P[4], float u, float z);
    
    // fields
    float _A[4];
    float _B[2];
    float _C[2];
    float _Q[4];
    float _R;

  private:

};


#endif
