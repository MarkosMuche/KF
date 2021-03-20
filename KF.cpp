
#include "Arduino.h"
#include "KF.h"


KF::KF() {
}

void KF::getABC(float  A[4], float B[2], float C[2]){
 this->_A[0]=A[0];
 this->_A[1]=A[1];
 this->_A[2]=A[2];
 this->_A[3]=A[3];
 this->_B[0]=B[0];
 this->_B[1]=B[1];
  this->_C[0]=C[0];
 this->_C[1]=C[1];
}

void KF::getQR(float Q[4], float R){
 this->_Q[0]=Q[0];
 this->_Q[1]=Q[1];
 this->_Q[2]=Q[2];
 this->_Q[3]=Q[3];
  this->_R=R;
}
//Kalmanfiltethis->_R
void KF::kalman(float x[2], float P[4],float u,float z)
{

    float  S;
    float L1;
    float  L2;
    float I1 = 1;
    float I2 = 1;
  float x1 = x[0];
  float x2 = x[1];
  float p11 = P[0];
  float p12 = P[1];
  float p21 = P[2];
  float p22 = P[3];
  //timeupdate
  float x1p = this->_A[0] * x1 + this->_A[1] * x2 + this->_B[0] * u;
  float x2p = this->_A[2] * x1 + this->_A[3] * x2;

  float p11p = (this->_A[0]) * (this->_A[0] * p11 + this->_A[1] * p21) + (this->_A[1]) * (this->_A[0] * p12 + this->_A[1] * p22)+this->_Q[1];
  float p12p = (this->_A[2]) * (this->_A[0] * p11 + this->_A[1] * p21) + (this->_A[3]) * (this->_A[0] * p12 + this->_A[1] * p22)+this->_Q[2];
  float p21p = (this->_A[0]) * (this->_A[2] * p11 + this->_A[3] * p21) + (this->_A[1]) * (this->_A[2] * p12 + this->_A[3] * p22)+this->_Q[3];
  float p22p = (this->_A[2]) * (this->_A[2] * p11 + this->_A[3] * p21) + (this->_A[3]) * (this->_A[2] * p12 + this->_A[3] * p22)+this->_Q[4];

  // calculate kalman gain
  S = (this->_C[0]) * (this->_C[0] * p11p + this->_C[1] * p21p) + (this->_C[1]) * (this->_C[0] * p12p + this->_C[1] * p22p) + this->_R ;
  S = 1 / S;
  L1 = (p11p * (this->_C[0]) + p12p * (this->_C[1])) * S;
  L2 = ( p21p * (this->_C[0]) + p22p * (this->_C[1])) * S;

  // time update
  x1 = x1p + L1 * (z - this->_C[0] * x1p - this->_C[1] * x2p);
  x2 = x2p + L2 * (z - this->_C[0] * x1p - this->_C[1] * x2p);

  p11 = (I1 - L1 * this->_C[0]) * p11p - L1 * this->_C[1] * p21p;
  p12 = (I1 - L1 * this->_C[0]) * p12p - L1 * this->_C[1] * p22p;
  p21 = -L2 * this->_C[0] * p11p + (I2 - L2 * this->_C[1]) * p21p;
  p22 = -L2 * this->_C[0] * p12p + (I2 - L2 * this->_C[1]) * p22p;

x[0]=x1;
x[1]=x2;
   float p11 = P[0];
  float p12 = P[1];
  float p21 = P[2];
  float p22 = P[3];
 P[0]=p11;
 P[1]=p12;
 P[2]=p21;
 P[3]=p22;
 
}
