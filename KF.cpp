
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
    //note that u is the gyro reading and z is the accelerometer reading
    float  S;
    float L1;
    float  L2;
    float I1 = 1;
    float I2 = 1;
    //time update
  float x1p = this->_A[0] * x[0] + this->_A[1] * x[1] + this->_B[0] * u;
  float x2p = this->_A[2] * x[0] + this->_A[3] * x[1]+this->_B[1]*u;
  
  float p11p = (this->_A[0]) * (this->_A[0] * P[0] + this->_A[1] * P[2]) + (this->_A[1]) * (this->_A[0] * P[1] + this->_A[1] * P[3])+this->_Q[0];
  float p12p = (this->_A[2]) * (this->_A[0] * P[0] + this->_A[1] * P[2]) + (this->_A[3]) * (this->_A[0] * P[1] + this->_A[1] * P[3])+this->_Q[1];
  float p21p = (this->_A[0]) * (this->_A[2] * P[0] + this->_A[3] * P[2]) + (this->_A[1]) * (this->_A[2] * P[1] + this->_A[3] * P[3])+this->_Q[2];
  float p22p = (this->_A[2]) * (this->_A[2] * P[0] + this->_A[3] * P[2]) + (this->_A[3]) * (this->_A[2] * P[1] + this->_A[3] * P[3])+this->_Q[3];
  // calculate kalman gain
  S = (this->_C[0]) * (this->_C[0] * p11p + this->_C[1] * p21p) + (this->_C[1]) * (this->_C[0] * p12p + this->_C[1] * p22p) + this->_R ;
  S = 1 / S;
  L1 = (p11p * this->_C[0] + p12p * this->_C[1]) * S;
  L2 = ( p21p * this->_C[0] + p22p * this->_C[1]) * S;
  
  // measuement update
  x[0] = x1p + L1 * (z - this->_C[0] * x1p - this->_C[1] * x2p);
  x[1] = x2p + L2 * (z - this->_C[0] * x1p - this->_C[1] * x2p);
 
  P[0] = (I1 - L1 * this->_C[0]) * p11p - L1 * this->_C[1] * p21p;
  P[1] = (I1 - L1 * this->_C[0]) * p12p - L1 * this->_C[1] * p22p;
  P[2] = -L2 * this->_C[0] * p11p + (I2 - L2 * this->_C[1]) * p21p;
  P[3] = -L2 * this->_C[0] * p12p + (I2 - L2 * this->_C[1]) * p22p;
}
