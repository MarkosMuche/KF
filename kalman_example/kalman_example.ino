
#include <KF.h>

KF kf;
float x[2] = {1, 1};
float P[4] = {0 , 1 , 1, 1};
float x[2] = {1, 1};
float P[4] = {0 , 1 , 1, 1};
float  A[4] = {1, 0.1, 0 , 1};
float B[2] = {0 , 1};
float C[2] = {1, 0};
float Q[4]={1, 0, 0, 10};
float R=1;

void setup() {
  Serial.begin(9600);
}

void loop() {
float r[2];
kf.getABC(A,B,C);
kf.kalman(x,P,1,1);
kf.getQR(Q,R);  
Serial.print(x[0]);
 Serial.print(x[1]);
   
}
