/*
 * Trial License - for use to evaluate programs for possible purchase as
 * an end-user only.
 *
 * esekf19_step.c
 *
 * Code generation for function 'esekf19_step'
 *
 */

/* Include files */
#include "esekf19_step.h"
#include "esekf19_app_data.h"
#include "norm.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
void inject_error(const double delta_state[19], double covariance[361],
                  double position[3], double velocity[3], double quaternion[4],
                  double accel_bias[3], double gyro_bias[3], double *scale,
                  double boresight[3])
{
  double G[361];
  double b_G[361];
  double delta_quaternion[4];
  double b_delta_quaternion_tmp;
  double b_quaternion;
  double c_delta_quaternion_tmp;
  double c_quaternion;
  double d_quaternion;
  double delta_quaternion_tmp;
  int G_tmp;
  int i;
  int i1;
  int i2;
  delta_quaternion[0] = 1.0;
  position[0] += delta_state[0];
  velocity[0] += delta_state[3];
  delta_quaternion_tmp = 0.5 * delta_state[6];
  delta_quaternion[1] = delta_quaternion_tmp;
  position[1] += delta_state[1];
  velocity[1] += delta_state[4];
  b_delta_quaternion_tmp = 0.5 * delta_state[7];
  delta_quaternion[2] = b_delta_quaternion_tmp;
  position[2] += delta_state[2];
  velocity[2] += delta_state[5];
  c_delta_quaternion_tmp = 0.5 * delta_state[8];
  delta_quaternion[3] = c_delta_quaternion_tmp;
  b_quaternion = b_norm(delta_quaternion);
  delta_quaternion[0] = 1.0 / b_quaternion;
  delta_quaternion[1] = delta_quaternion_tmp / b_quaternion;
  delta_quaternion[2] = b_delta_quaternion_tmp / b_quaternion;
  delta_quaternion[3] = c_delta_quaternion_tmp / b_quaternion;
  b_quaternion = quaternion[0];
  c_quaternion = quaternion[1];
  d_quaternion = quaternion[2];
  quaternion[0] = ((quaternion[0] * delta_quaternion[0] -
                    quaternion[1] * delta_quaternion[1]) -
                   quaternion[2] * delta_quaternion[2]) -
                  quaternion[3] * delta_quaternion[3];
  quaternion[1] = ((b_quaternion * delta_quaternion[1] +
                    delta_quaternion[0] * quaternion[1]) +
                   quaternion[2] * delta_quaternion[3]) -
                  delta_quaternion[2] * quaternion[3];
  quaternion[2] = ((b_quaternion * delta_quaternion[2] -
                    c_quaternion * delta_quaternion[3]) +
                   delta_quaternion[0] * quaternion[2]) +
                  delta_quaternion[1] * quaternion[3];
  quaternion[3] = ((b_quaternion * delta_quaternion[3] +
                    c_quaternion * delta_quaternion[2]) -
                   delta_quaternion[1] * d_quaternion) +
                  delta_quaternion[0] * quaternion[3];
  b_quaternion = b_norm(quaternion);
  quaternion[0] /= b_quaternion;
  quaternion[1] /= b_quaternion;
  quaternion[2] /= b_quaternion;
  quaternion[3] /= b_quaternion;
  *scale += delta_state[15];
  accel_bias[0] += delta_state[9];
  gyro_bias[0] += delta_state[12];
  boresight[0] += delta_state[16];
  accel_bias[1] += delta_state[10];
  gyro_bias[1] += delta_state[13];
  boresight[1] += delta_state[17];
  accel_bias[2] += delta_state[11];
  gyro_bias[2] += delta_state[14];
  boresight[2] += delta_state[18];
  for (i = 0; i < 361; i++) {
    G[i] = iv[i];
  }
  G[120] = 1.0;
  G[121] = 0.0 - c_delta_quaternion_tmp;
  G[122] = 0.0 - 0.5 * -delta_state[7];
  G[139] = 0.0 - 0.5 * -delta_state[8];
  G[140] = 1.0;
  G[141] = 0.0 - delta_quaternion_tmp;
  G[158] = 0.0 - b_delta_quaternion_tmp;
  G[159] = 0.0 - 0.5 * -delta_state[6];
  G[160] = 1.0;
  memset(&b_G[0], 0, 361U * sizeof(double));
  for (i = 0; i < 19; i++) {
    for (i1 = 0; i1 < 19; i1++) {
      G_tmp = i1 + 19 * i;
      b_quaternion = covariance[G_tmp];
      for (i2 = 0; i2 < 19; i2++) {
        int b_G_tmp;
        b_G_tmp = i2 + 19 * i;
        b_G[b_G_tmp] += G[i2 + 19 * i1] * b_quaternion;
      }
      covariance[G_tmp] = 0.0;
    }
  }
  for (i = 0; i < 19; i++) {
    for (i1 = 0; i1 < 19; i1++) {
      b_quaternion = G[i + 19 * i1];
      for (i2 = 0; i2 < 19; i2++) {
        G_tmp = i2 + 19 * i;
        covariance[G_tmp] += b_G[i2 + 19 * i1] * b_quaternion;
      }
    }
  }
  for (i = 0; i < 19; i++) {
    for (i1 = 0; i1 < 19; i1++) {
      G_tmp = i1 + 19 * i;
      G[G_tmp] = 0.5 * (covariance[G_tmp] + covariance[i + 19 * i1]);
    }
  }
  memcpy(&covariance[0], &G[0], 361U * sizeof(double));
}

/* End of code generation (esekf19_step.c) */
