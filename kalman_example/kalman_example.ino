
#include <KF.h>

KF kf;
float x[2] = {0.1, 0.1};
float P[4] = {0.1 , 0, 0, 0.1};
float  A[4] = {1, 0.1, 0 , 1};
float B[2] = {0 , 1};
float C[2] = {1, 0};
float Q[4] = {1, 0, 0, 10};
float R = 1;
void setup() {
  Serial.begin(9600);
  kf.getABC(A, B, C);
  kf.getQR(Q, R);
}

void loop() {
  float z = 1;
  float u = 1;
  kf.kalman(x, P, u, z);
  Serial.print(x[0]);
  Serial.print(x[1]);
}
