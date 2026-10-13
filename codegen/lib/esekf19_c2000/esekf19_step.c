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
#include "atan2.h"
#include "diag.h"
#include "esekf19_c2000_data.h"
#include "eye.h"
#include "mldivide.h"
#include "mrdivide_helper.h"
#include "norm.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Variable Definitions */
static const int iv1[361] = {
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1};

/* Function Declarations */
static void inject_error(const double delta_state[19], double covariance[361],
                         double position[3], double velocity[3],
                         double quaternion[4], double accel_bias[3],
                         double gyro_bias[3], double *scale,
                         double boresight[3]);

/* Function Definitions */
static void inject_error(const double delta_state[19], double covariance[361],
                         double position[3], double velocity[3],
                         double quaternion[4], double accel_bias[3],
                         double gyro_bias[3], double *scale,
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
    G[i] = iv1[i];
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

void esekf19_step(double state[20], double covariance[361],
                  const double process_noise[361], const double gyro_meas[3],
                  const double accel_meas[3], const double gnss_meas[7],
                  double depth_meas, const double dvl_meas[3],
                  const double measurement_flags[3], const double cfg[18],
                  double update_status[3], double nis[3])
{
  static const int b_H[133] = {
      1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
      0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  static const int b_iv1[133] = {
      1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  static const int H[19] = {0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0};
  static const int b_iv[19] = {0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0, 0, 0, 0};
  static const int a[9] = {-1, 0, 0, 0, -1, 0, 0, 0, -1};
  double A[361];
  double F[361];
  double b_F[361];
  double b_K[133];
  double c_K[133];
  double c_H[57];
  double d_H[57];
  double delta_state[19];
  double Cbn[9];
  double Cnb[9];
  double b_S[9];
  double dv[9];
  double residual[7];
  double quaternion[4];
  double accel_bias[3];
  double accel_corrected[3];
  double boresight[3];
  double gyro_bias[3];
  double gyro_corrected[3];
  double position[3];
  double velocity[3];
  double velocity_at_dvl[3];
  double w_en_ned[3];
  double w_nb_body[3];
  double Cbn_tmp;
  double S;
  double angle_norm;
  double b_Cbn_tmp;
  double b_w_en_ned_tmp;
  double c_Cbn_tmp;
  double cfg_idx_0;
  double delta_quaternion_idx_0;
  double delta_quaternion_idx_1;
  double delta_quaternion_idx_2;
  double delta_quaternion_idx_3;
  double scale;
  double w_en_ned_tmp;
  int Cnb_tmp;
  int F_tmp;
  int b_F_tmp;
  int b_i;
  int c_i;
  int i;
  bool b;
  /* ESEKF19_STEP One fixed-size INS prediction and optional sensor updates. */
  /*  Code-generation entry point: no plotting, RNG, variable-size data or I/O.
   */
  /*  state (20x1): p[1:3], v[4:6], quaternion wxyz[7:10], ba[11:13], */
  /*  bg[14:16], scale[17], boresight alpha/beta/gamma[18:20]. */
  /*  covariance/process_noise: fixed 19x19 error-state matrices. */
  /*  measurement_flags: [GNSS_valid; depth_valid; DVL_valid], fixed 3x1. */
  /*  gnss_meas (7x1): position NED, velocity NED, yaw (rad). */
  /*  cfg (18x1): dt, gravity, w_ie_NED[3], RN, RM, tan(latitude), height, */
  /*  lever_arm_body[3], DVL noise floor/slope, GPS pos/vel/yaw sigma, depth
   * sigma. */
  /*  viết code comment vào  */
  scale = state[16];
  accel_bias[0] = state[10];
  gyro_bias[0] = state[13];
  boresight[0] = state[17];
  accel_bias[1] = state[11];
  gyro_bias[1] = state[14];
  boresight[1] = state[18];
  accel_bias[2] = state[12];
  gyro_bias[2] = state[15];
  boresight[2] = state[19];
  /*  INS mechanization prediction in the local NED frame. */
  angle_norm = b_norm(&state[6]);
  quaternion[0] = state[6] / angle_norm;
  quaternion[1] = state[7] / angle_norm;
  quaternion[2] = state[8] / angle_norm;
  quaternion[3] = state[9] / angle_norm;
  Cbn_tmp = quaternion[3] * quaternion[3];
  delta_quaternion_idx_3 = quaternion[2] * quaternion[2];
  Cbn[0] = 1.0 - 2.0 * (delta_quaternion_idx_3 + Cbn_tmp);
  angle_norm = quaternion[1] * quaternion[2];
  S = quaternion[0] * quaternion[3];
  Cbn[3] = 2.0 * (angle_norm - S);
  b_Cbn_tmp = quaternion[1] * quaternion[3];
  c_Cbn_tmp = quaternion[0] * quaternion[2];
  Cbn[6] = 2.0 * (b_Cbn_tmp + c_Cbn_tmp);
  Cbn[1] = 2.0 * (angle_norm + S);
  S = quaternion[1] * quaternion[1];
  Cbn[4] = 1.0 - 2.0 * (S + Cbn_tmp);
  angle_norm = quaternion[2] * quaternion[3];
  Cbn_tmp = quaternion[0] * quaternion[1];
  Cbn[7] = 2.0 * (angle_norm - Cbn_tmp);
  Cbn[2] = 2.0 * (b_Cbn_tmp - c_Cbn_tmp);
  Cbn[5] = 2.0 * (angle_norm + Cbn_tmp);
  Cbn[8] = 1.0 - 2.0 * (S + delta_quaternion_idx_3);
  w_en_ned_tmp = cfg[5] + cfg[8];
  w_en_ned[0] = state[4] / w_en_ned_tmp;
  b_w_en_ned_tmp = cfg[6] + cfg[8];
  w_en_ned[1] = -state[3] / b_w_en_ned_tmp;
  w_en_ned[2] = -state[4] * cfg[7] / w_en_ned_tmp;
  accel_corrected[0] = accel_meas[0] - state[10];
  gyro_corrected[0] = gyro_meas[0] - state[13];
  cfg_idx_0 = w_en_ned[0] + cfg[2];
  accel_corrected[1] = accel_meas[1] - state[11];
  gyro_corrected[1] = gyro_meas[1] - state[14];
  delta_quaternion_idx_0 = w_en_ned[1] + cfg[3];
  accel_corrected[2] = accel_meas[2] - state[12];
  gyro_corrected[2] = gyro_meas[2] - state[15];
  delta_quaternion_idx_1 = w_en_ned[2] + cfg[4];
  memset(&velocity_at_dvl[0], 0, 3U * sizeof(double));
  Cbn_tmp = velocity_at_dvl[0];
  angle_norm = velocity_at_dvl[1];
  S = velocity_at_dvl[2];
  for (i = 0; i < 3; i++) {
    delta_quaternion_idx_3 = Cbn[3 * i];
    b_Cbn_tmp = delta_quaternion_idx_3 * cfg_idx_0;
    c_Cbn_tmp = accel_corrected[i];
    Cbn_tmp += delta_quaternion_idx_3 * c_Cbn_tmp;
    delta_quaternion_idx_3 = Cbn[3 * i + 1];
    b_Cbn_tmp += delta_quaternion_idx_3 * delta_quaternion_idx_0;
    angle_norm += delta_quaternion_idx_3 * c_Cbn_tmp;
    delta_quaternion_idx_3 = Cbn[3 * i + 2];
    b_Cbn_tmp += delta_quaternion_idx_3 * delta_quaternion_idx_1;
    S += delta_quaternion_idx_3 * c_Cbn_tmp;
    w_nb_body[i] = gyro_corrected[i] - b_Cbn_tmp;
    w_en_ned[i] += 2.0 * cfg[i + 2];
  }
  velocity[0] =
      state[3] +
      (Cbn_tmp - (w_en_ned[1] * state[5] - w_en_ned[2] * state[4])) * cfg[0];
  velocity[1] =
      state[4] +
      (angle_norm - (w_en_ned[2] * state[3] - w_en_ned[0] * state[5])) * cfg[0];
  velocity[2] = state[5] + ((cfg[1] + S) -
                            (w_en_ned[0] * state[4] - w_en_ned[1] * state[3])) *
                               cfg[0];
  position[0] = state[0] + velocity[0] * cfg[0];
  b_Cbn_tmp = w_nb_body[0] * cfg[0];
  w_en_ned[0] = b_Cbn_tmp;
  position[1] = state[1] + cfg[0] * velocity[1];
  c_Cbn_tmp = cfg[0] * w_nb_body[1];
  w_en_ned[1] = c_Cbn_tmp;
  position[2] = state[2] + cfg[0] * velocity[2];
  cfg_idx_0 = cfg[0] * w_nb_body[2];
  w_en_ned[2] = cfg_idx_0;
  angle_norm = c_norm(w_en_ned);
  Cbn_tmp = angle_norm / 2.0;
  S = sin(Cbn_tmp);
  angle_norm = fmax(angle_norm, 1.0E-12);
  delta_quaternion_idx_0 = cos(Cbn_tmp);
  delta_quaternion_idx_1 = S * b_Cbn_tmp / angle_norm;
  delta_quaternion_idx_2 = S * c_Cbn_tmp / angle_norm;
  delta_quaternion_idx_3 = S * cfg_idx_0 / angle_norm;
  quaternion[0] =
      ((delta_quaternion_idx_0 * state[6] - delta_quaternion_idx_1 * state[7]) -
       delta_quaternion_idx_2 * state[8]) -
      delta_quaternion_idx_3 * state[9];
  quaternion[1] =
      ((delta_quaternion_idx_1 * state[6] + delta_quaternion_idx_0 * state[7]) +
       delta_quaternion_idx_3 * state[8]) -
      delta_quaternion_idx_2 * state[9];
  quaternion[2] =
      ((delta_quaternion_idx_2 * state[6] - delta_quaternion_idx_3 * state[7]) +
       delta_quaternion_idx_0 * state[8]) +
      delta_quaternion_idx_1 * state[9];
  quaternion[3] =
      ((delta_quaternion_idx_3 * state[6] + delta_quaternion_idx_2 * state[7]) -
       delta_quaternion_idx_1 * state[8]) +
      delta_quaternion_idx_0 * state[9];
  angle_norm = b_norm(quaternion);
  quaternion[0] /= angle_norm;
  quaternion[1] /= angle_norm;
  quaternion[2] /= angle_norm;
  quaternion[3] /= angle_norm;
  for (i = 0; i < 361; i++) {
    F[i] = iv1[i];
  }
  angle_norm = cfg[0];
  for (i = 0; i < 3; i++) {
    F_tmp = 19 * (i + 3);
    F[F_tmp] = (double)iv[3 * i] * angle_norm;
    F[F_tmp + 1] = (double)iv[3 * i + 1] * angle_norm;
    F[F_tmp + 2] = (double)iv[3 * i + 2] * angle_norm;
  }
  for (i = 0; i < 9; i++) {
    b_S[i] = -Cbn[i];
  }
  dv[0] = 0.0;
  dv[3] = -accel_corrected[2];
  dv[6] = accel_corrected[1];
  dv[1] = accel_corrected[2];
  dv[4] = 0.0;
  dv[7] = -accel_corrected[0];
  dv[2] = -accel_corrected[1];
  dv[5] = accel_corrected[0];
  dv[8] = 0.0;
  memset(&Cnb[0], 0, 9U * sizeof(double));
  for (b_i = 0; b_i < 3; b_i++) {
    angle_norm = Cnb[3 * b_i];
    F_tmp = 3 * b_i + 1;
    Cnb_tmp = 3 * b_i + 2;
    for (i = 0; i < 3; i++) {
      S = dv[i + 3 * b_i];
      angle_norm += b_S[3 * i] * S;
      Cnb[F_tmp] += b_S[3 * i + 1] * S;
      Cnb[Cnb_tmp] += b_S[3 * i + 2] * S;
    }
    Cnb[3 * b_i] = angle_norm;
    Cnb_tmp = 19 * (b_i + 6);
    F[Cnb_tmp + 3] = angle_norm * cfg[0];
    b_F_tmp = 19 * (b_i + 9);
    F[b_F_tmp + 3] = -Cbn[3 * b_i] * cfg[0];
    F_tmp = 3 * b_i + 1;
    F[Cnb_tmp + 4] = Cnb[F_tmp] * cfg[0];
    F[b_F_tmp + 4] = -Cbn[F_tmp] * cfg[0];
    F_tmp = 3 * b_i + 2;
    F[Cnb_tmp + 5] = Cnb[F_tmp] * cfg[0];
    F[b_F_tmp + 5] = -Cbn[F_tmp] * cfg[0];
  }
  eye(b_S);
  angle_norm = 0.0 * cfg[0];
  accel_corrected[0] = angle_norm;
  accel_corrected[1] = -w_nb_body[2] * cfg[0];
  accel_corrected[2] = c_Cbn_tmp;
  velocity_at_dvl[0] = cfg_idx_0;
  velocity_at_dvl[1] = angle_norm;
  velocity_at_dvl[2] = -w_nb_body[0] * cfg[0];
  w_en_ned[0] = -w_nb_body[1] * cfg[0];
  w_en_ned[1] = b_Cbn_tmp;
  w_en_ned[2] = angle_norm;
  angle_norm = cfg[0];
  for (i = 0; i < 3; i++) {
    F_tmp = 19 * (i + 6);
    F[F_tmp + 6] = b_S[3 * i] - accel_corrected[i];
    Cnb_tmp = 3 * i + 1;
    F[F_tmp + 7] = b_S[Cnb_tmp] - velocity_at_dvl[i];
    b_F_tmp = 3 * i + 2;
    F[F_tmp + 8] = b_S[b_F_tmp] - w_en_ned[i];
    F_tmp = 19 * (i + 12);
    F[F_tmp + 6] = (double)a[3 * i] * angle_norm;
    F[F_tmp + 7] = (double)a[Cnb_tmp] * angle_norm;
    F[F_tmp + 8] = (double)a[b_F_tmp] * angle_norm;
  }
  memset(&b_F[0], 0, 361U * sizeof(double));
  for (i = 0; i < 19; i++) {
    for (c_i = 0; c_i < 19; c_i++) {
      angle_norm = covariance[c_i + 19 * i];
      for (b_i = 0; b_i < 19; b_i++) {
        F_tmp = b_i + 19 * i;
        b_F[F_tmp] += F[b_i + 19 * c_i] * angle_norm;
      }
    }
  }
  for (i = 0; i < 19; i++) {
    for (b_i = 0; b_i < 19; b_i++) {
      angle_norm = 0.0;
      for (c_i = 0; c_i < 19; c_i++) {
        angle_norm += b_F[i + 19 * c_i] * F[b_i + 19 * c_i];
      }
      F_tmp = i + 19 * b_i;
      covariance[F_tmp] = angle_norm + process_noise[F_tmp];
    }
  }
  update_status[0] = 0.0;
  nis[0] = 0.0;
  update_status[1] = 0.0;
  nis[1] = 0.0;
  update_status[2] = 0.0;
  nis[2] = 0.0;
  if (measurement_flags[0] != 0.0) {
    double R[49];
    double c_S[49];
    double b_a[7];
    angle_norm = b_norm(quaternion);
    delta_quaternion_idx_2 = quaternion[2] / angle_norm;
    delta_quaternion_idx_3 = quaternion[3] / angle_norm;
    angle_norm =
        b_atan2(2.0 * (quaternion[1] / angle_norm * delta_quaternion_idx_2 +
                       quaternion[0] / angle_norm * delta_quaternion_idx_3),
                1.0 - 2.0 * (delta_quaternion_idx_2 * delta_quaternion_idx_2 +
                             delta_quaternion_idx_3 * delta_quaternion_idx_3));
    S = gnss_meas[6] - angle_norm;
    angle_norm = b_atan2(sin(S), cos(S));
    residual[6] = angle_norm;
    angle_norm = cfg[14] * cfg[14];
    S = cfg[15] * cfg[15];
    residual[0] = gnss_meas[0] - position[0];
    residual[3] = gnss_meas[3] - velocity[0];
    b_a[0] = angle_norm;
    b_a[3] = S;
    residual[1] = gnss_meas[1] - position[1];
    residual[4] = gnss_meas[4] - velocity[1];
    b_a[1] = angle_norm;
    b_a[4] = S;
    residual[2] = gnss_meas[2] - position[2];
    residual[5] = gnss_meas[5] - velocity[2];
    b_a[2] = angle_norm;
    b_a[5] = S;
    b_a[6] = cfg[16] * cfg[16];
    b_diag(b_a, R);
    memset(&b_K[0], 0, 133U * sizeof(double));
    for (i = 0; i < 19; i++) {
      for (c_i = 0; c_i < 19; c_i++) {
        angle_norm = covariance[c_i + 19 * i];
        for (b_i = 0; b_i < 7; b_i++) {
          F_tmp = b_i + 7 * i;
          b_K[F_tmp] += (double)b_H[b_i + 7 * c_i] * angle_norm;
        }
      }
    }
    for (c_i = 0; c_i < 7; c_i++) {
      for (b_i = 0; b_i < 7; b_i++) {
        angle_norm = 0.0;
        for (i = 0; i < 19; i++) {
          angle_norm += b_K[c_i + 7 * i] * (double)b_iv1[i + 19 * b_i];
        }
        F_tmp = c_i + 7 * b_i;
        c_S[F_tmp] = angle_norm + R[F_tmp];
      }
      b_a[c_i] = residual[c_i];
    }
    b_mldivide(c_S, b_a);
    Cbn_tmp = 0.0;
    for (i = 0; i < 7; i++) {
      Cbn_tmp += residual[i] * b_a[i];
    }
    if (!rtIsInf(Cbn_tmp) && !rtIsNaN(Cbn_tmp) && (Cbn_tmp <= 18.48)) {
      b = true;
    } else {
      b = false;
    }
    memset(&delta_state[0], 0, 19U * sizeof(double));
    if (b) {
      memset(&b_K[0], 0, 133U * sizeof(double));
      for (i = 0; i < 7; i++) {
        for (b_i = 0; b_i < 19; b_i++) {
          F_tmp = b_iv1[b_i + 19 * i];
          for (c_i = 0; c_i < 19; c_i++) {
            Cnb_tmp = c_i + 19 * i;
            b_K[Cnb_tmp] += covariance[c_i + 19 * b_i] * (double)F_tmp;
          }
        }
      }
      b_mrdiv(b_K, c_S);
      memset(&delta_state[0], 0, 19U * sizeof(double));
      for (i = 0; i < 7; i++) {
        angle_norm = residual[i];
        for (c_i = 0; c_i < 19; c_i++) {
          delta_state[c_i] += b_K[c_i + 19 * i] * angle_norm;
        }
      }
      b_eye(F);
      for (b_i = 0; b_i < 19; b_i++) {
        for (c_i = 0; c_i < 19; c_i++) {
          angle_norm = 0.0;
          for (i = 0; i < 7; i++) {
            angle_norm += b_K[b_i + 19 * i] * (double)b_H[i + 7 * c_i];
          }
          F_tmp = b_i + 19 * c_i;
          A[F_tmp] = F[F_tmp] - angle_norm;
        }
      }
      memset(&b_F[0], 0, 361U * sizeof(double));
      for (i = 0; i < 19; i++) {
        for (b_i = 0; b_i < 19; b_i++) {
          angle_norm = covariance[b_i + 19 * i];
          for (c_i = 0; c_i < 19; c_i++) {
            F_tmp = c_i + 19 * i;
            b_F[F_tmp] += A[c_i + 19 * b_i] * angle_norm;
          }
        }
      }
      memset(&c_K[0], 0, 133U * sizeof(double));
      for (i = 0; i < 7; i++) {
        for (b_i = 0; b_i < 7; b_i++) {
          angle_norm = R[b_i + 7 * i];
          for (c_i = 0; c_i < 19; c_i++) {
            F_tmp = c_i + 19 * i;
            c_K[F_tmp] += b_K[c_i + 19 * b_i] * angle_norm;
          }
        }
      }
      memset(&covariance[0], 0, 361U * sizeof(double));
      for (i = 0; i < 19; i++) {
        for (b_i = 0; b_i < 19; b_i++) {
          angle_norm = A[i + 19 * b_i];
          for (c_i = 0; c_i < 19; c_i++) {
            F_tmp = c_i + 19 * i;
            covariance[F_tmp] += b_F[c_i + 19 * b_i] * angle_norm;
          }
        }
      }
      memset(&b_F[0], 0, 361U * sizeof(double));
      for (i = 0; i < 19; i++) {
        for (b_i = 0; b_i < 7; b_i++) {
          angle_norm = b_K[i + 19 * b_i];
          for (c_i = 0; c_i < 19; c_i++) {
            F_tmp = c_i + 19 * i;
            b_F[F_tmp] += c_K[c_i + 19 * b_i] * angle_norm;
          }
        }
      }
      for (i = 0; i < 361; i++) {
        covariance[i] += b_F[i];
      }
      for (i = 0; i < 19; i++) {
        for (c_i = 0; c_i < 19; c_i++) {
          F_tmp = c_i + 19 * i;
          b_F[F_tmp] = 0.5 * (covariance[F_tmp] + covariance[i + 19 * c_i]);
        }
      }
      memcpy(&covariance[0], &b_F[0], 361U * sizeof(double));
    }
    update_status[0] = b;
    nis[0] = Cbn_tmp;
    if (b) {
      accel_bias[0] = state[10];
      gyro_bias[0] = state[13];
      boresight[0] = state[17];
      accel_bias[1] = state[11];
      gyro_bias[1] = state[14];
      boresight[1] = state[18];
      accel_bias[2] = state[12];
      gyro_bias[2] = state[15];
      boresight[2] = state[19];
      inject_error(delta_state, covariance, position, velocity, quaternion,
                   accel_bias, gyro_bias, &scale, boresight);
    }
  }
  if (measurement_flags[1] != 0.0) {
    Cbn_tmp = depth_meas - position[2];
    b_Cbn_tmp = cfg[17] * cfg[17];
    memset(&delta_state[0], 0, 19U * sizeof(double));
    angle_norm = 0.0;
    for (i = 0; i < 19; i++) {
      delta_quaternion_idx_3 = delta_state[i];
      for (c_i = 0; c_i < 19; c_i++) {
        delta_quaternion_idx_3 += (double)H[c_i] * covariance[c_i + 19 * i];
      }
      delta_state[i] = delta_quaternion_idx_3;
      angle_norm += delta_quaternion_idx_3 * (double)b_iv[i];
    }
    S = angle_norm + b_Cbn_tmp;
    delta_quaternion_idx_3 = Cbn_tmp * (Cbn_tmp / S);
    if (!rtIsInf(delta_quaternion_idx_3) && !rtIsNaN(delta_quaternion_idx_3) &&
        (delta_quaternion_idx_3 <= 6.63)) {
      b = true;
    } else {
      b = false;
    }
    memset(&delta_state[0], 0, 19U * sizeof(double));
    if (b) {
      double K[19];
      for (i = 0; i < 19; i++) {
        angle_norm = 0.0;
        for (c_i = 0; c_i < 19; c_i++) {
          angle_norm += covariance[i + 19 * c_i] * (double)b_iv[c_i];
        }
        angle_norm /= S;
        K[i] = angle_norm;
        delta_state[i] = angle_norm * Cbn_tmp;
      }
      b_eye(F);
      for (i = 0; i < 19; i++) {
        F_tmp = H[i];
        for (b_i = 0; b_i < 19; b_i++) {
          Cnb_tmp = b_i + 19 * i;
          A[Cnb_tmp] = F[Cnb_tmp] - K[b_i] * (double)F_tmp;
        }
      }
      memset(&b_F[0], 0, 361U * sizeof(double));
      for (i = 0; i < 19; i++) {
        for (b_i = 0; b_i < 19; b_i++) {
          angle_norm = covariance[b_i + 19 * i];
          for (c_i = 0; c_i < 19; c_i++) {
            F_tmp = c_i + 19 * i;
            b_F[F_tmp] += A[c_i + 19 * b_i] * angle_norm;
          }
        }
      }
      memset(&covariance[0], 0, 361U * sizeof(double));
      for (i = 0; i < 19; i++) {
        for (b_i = 0; b_i < 19; b_i++) {
          angle_norm = A[i + 19 * b_i];
          for (c_i = 0; c_i < 19; c_i++) {
            F_tmp = c_i + 19 * i;
            covariance[F_tmp] += b_F[c_i + 19 * b_i] * angle_norm;
          }
        }
      }
      for (c_i = 0; c_i < 19; c_i++) {
        for (i = 0; i < 19; i++) {
          b_F[i + 19 * c_i] = K[i] * b_Cbn_tmp * K[c_i];
        }
      }
      for (i = 0; i < 361; i++) {
        covariance[i] += b_F[i];
      }
      for (i = 0; i < 19; i++) {
        for (c_i = 0; c_i < 19; c_i++) {
          F_tmp = c_i + 19 * i;
          b_F[F_tmp] = 0.5 * (covariance[F_tmp] + covariance[i + 19 * c_i]);
        }
      }
      memcpy(&covariance[0], &b_F[0], 361U * sizeof(double));
    }
    update_status[1] = b;
    nis[1] = delta_quaternion_idx_3;
    if (b) {
      inject_error(delta_state, covariance, position, velocity, quaternion,
                   accel_bias, gyro_bias, &scale, boresight);
    }
  }
  if (measurement_flags[2] != 0.0) {
    double d_K[57];
    angle_norm = b_norm(quaternion);
    delta_quaternion_idx_0 = quaternion[0] / angle_norm;
    delta_quaternion_idx_1 = quaternion[1] / angle_norm;
    delta_quaternion_idx_2 = quaternion[2] / angle_norm;
    delta_quaternion_idx_3 = quaternion[3] / angle_norm;
    S = delta_quaternion_idx_3 * delta_quaternion_idx_3;
    b_Cbn_tmp = delta_quaternion_idx_2 * delta_quaternion_idx_2;
    Cnb[0] = 1.0 - 2.0 * (b_Cbn_tmp + S);
    Cbn_tmp = delta_quaternion_idx_1 * delta_quaternion_idx_2;
    angle_norm = delta_quaternion_idx_0 * delta_quaternion_idx_3;
    Cnb[1] = 2.0 * (Cbn_tmp - angle_norm);
    c_Cbn_tmp = delta_quaternion_idx_1 * delta_quaternion_idx_3;
    cfg_idx_0 = delta_quaternion_idx_0 * delta_quaternion_idx_2;
    Cnb[2] = 2.0 * (c_Cbn_tmp + cfg_idx_0);
    Cnb[3] = 2.0 * (Cbn_tmp + angle_norm);
    angle_norm = delta_quaternion_idx_1 * delta_quaternion_idx_1;
    Cnb[4] = 1.0 - 2.0 * (angle_norm + S);
    Cbn_tmp = delta_quaternion_idx_2 * delta_quaternion_idx_3;
    delta_quaternion_idx_3 = delta_quaternion_idx_0 * delta_quaternion_idx_1;
    Cnb[5] = 2.0 * (Cbn_tmp - delta_quaternion_idx_3);
    Cnb[6] = 2.0 * (c_Cbn_tmp - cfg_idx_0);
    Cnb[7] = 2.0 * (Cbn_tmp + delta_quaternion_idx_3);
    Cnb[8] = 1.0 - 2.0 * (angle_norm + b_Cbn_tmp);
    memset(&w_nb_body[0], 0, 3U * sizeof(double));
    cfg_idx_0 = cfg[2] + velocity[1] / w_en_ned_tmp;
    delta_quaternion_idx_0 = cfg[3] + -velocity[0] / b_w_en_ned_tmp;
    delta_quaternion_idx_1 = cfg[4] + -velocity[1] * cfg[7] / w_en_ned_tmp;
    delta_quaternion_idx_3 = w_nb_body[0];
    b_Cbn_tmp = w_nb_body[1];
    c_Cbn_tmp = w_nb_body[2];
    for (i = 0; i < 3; i++) {
      angle_norm = velocity[i];
      delta_quaternion_idx_3 += Cnb[3 * i] * angle_norm;
      b_Cbn_tmp += Cnb[3 * i + 1] * angle_norm;
      c_Cbn_tmp += Cnb[3 * i + 2] * angle_norm;
      w_en_ned[i] =
          gyro_corrected[i] -
          ((Cnb[i] * cfg_idx_0 + Cnb[i + 3] * delta_quaternion_idx_0) +
           Cnb[i + 6] * delta_quaternion_idx_1);
    }
    velocity_at_dvl[0] = delta_quaternion_idx_3 +
                         (w_en_ned[1] * cfg[11] - w_en_ned[2] * cfg[10]);
    velocity_at_dvl[1] =
        b_Cbn_tmp + (w_en_ned[2] * cfg[9] - w_en_ned[0] * cfg[11]);
    velocity_at_dvl[2] =
        c_Cbn_tmp + (w_en_ned[0] * cfg[10] - w_en_ned[1] * cfg[9]);
    b_S[0] = 1.0;
    b_S[1] = 0.0 - boresight[2];
    b_S[2] = 0.0 - (-boresight[1]);
    b_S[3] = 0.0 - (-boresight[2]);
    b_S[4] = 1.0;
    b_S[5] = 0.0 - boresight[0];
    b_S[6] = 0.0 - boresight[1];
    b_S[7] = 0.0 - (-boresight[0]);
    b_S[8] = 1.0;
    for (i = 0; i < 9; i++) {
      Cbn[i] = (scale + 1.0) * b_S[i];
    }
    angle_norm = velocity_at_dvl[0];
    S = velocity_at_dvl[1];
    Cbn_tmp = velocity_at_dvl[2];
    for (i = 0; i < 3; i++) {
      accel_corrected[i] =
          dvl_meas[i] -
          ((Cbn[i] * angle_norm + Cbn[i + 3] * S) + Cbn[i + 6] * Cbn_tmp);
    }
    memset(&c_H[0], 0, 57U * sizeof(double));
    dv[0] = 0.0;
    dv[3] = -c_Cbn_tmp;
    dv[6] = b_Cbn_tmp;
    dv[1] = c_Cbn_tmp;
    dv[4] = 0.0;
    dv[7] = -delta_quaternion_idx_3;
    dv[2] = -b_Cbn_tmp;
    dv[5] = delta_quaternion_idx_3;
    dv[8] = 0.0;
    for (b_i = 0; b_i < 3; b_i++) {
      F_tmp = 3 * (b_i + 3);
      c_H[F_tmp] = 0.0;
      c_H[F_tmp + 1] = 0.0;
      c_H[F_tmp + 2] = 0.0;
      for (i = 0; i < 3; i++) {
        angle_norm = Cnb[i + 3 * b_i];
        c_H[F_tmp] += Cbn[3 * i] * angle_norm;
        c_H[F_tmp + 1] += Cbn[3 * i + 1] * angle_norm;
        c_H[F_tmp + 2] += Cbn[3 * i + 2] * angle_norm;
        c_H[i + 3 * (b_i + 6)] = 0.0;
      }
      F_tmp = 3 * (b_i + 6);
      for (i = 0; i < 3; i++) {
        angle_norm = dv[i + 3 * b_i];
        c_H[F_tmp] += Cbn[3 * i] * angle_norm;
        c_H[F_tmp + 1] += Cbn[3 * i + 1] * angle_norm;
        c_H[F_tmp + 2] += Cbn[3 * i + 2] * angle_norm;
        c_H[i + 3 * (b_i + 12)] = 0.0;
      }
    }
    dv[0] = 0.0;
    dv[3] = -cfg[11];
    dv[6] = cfg[10];
    dv[1] = cfg[11];
    dv[4] = 0.0;
    dv[7] = -cfg[9];
    dv[2] = -cfg[10];
    dv[5] = cfg[9];
    dv[8] = 0.0;
    for (i = 0; i < 3; i++) {
      F_tmp = 3 * (i + 12);
      for (c_i = 0; c_i < 3; c_i++) {
        angle_norm = dv[c_i + 3 * i];
        c_H[F_tmp] += Cbn[3 * c_i] * angle_norm;
        c_H[F_tmp + 1] += Cbn[3 * c_i + 1] * angle_norm;
        c_H[F_tmp + 2] += Cbn[3 * c_i + 2] * angle_norm;
      }
      c_H[i + 45] = 0.0;
    }
    angle_norm = c_H[45];
    S = c_H[46];
    Cbn_tmp = c_H[47];
    for (i = 0; i < 3; i++) {
      delta_quaternion_idx_3 = velocity_at_dvl[i];
      angle_norm += b_S[3 * i] * delta_quaternion_idx_3;
      S += b_S[3 * i + 1] * delta_quaternion_idx_3;
      Cbn_tmp += b_S[3 * i + 2] * delta_quaternion_idx_3;
    }
    c_H[47] = Cbn_tmp;
    c_H[46] = S;
    c_H[45] = angle_norm;
    angle_norm = (scale + 1.0) * 0.0;
    c_H[48] = angle_norm;
    c_H[51] = (scale + 1.0) * -velocity_at_dvl[2];
    c_H[54] = (scale + 1.0) * velocity_at_dvl[1];
    c_H[49] = (scale + 1.0) * velocity_at_dvl[2];
    c_H[52] = angle_norm;
    c_H[55] = (scale + 1.0) * -velocity_at_dvl[0];
    c_H[50] = (scale + 1.0) * -velocity_at_dvl[1];
    c_H[53] = (scale + 1.0) * velocity_at_dvl[0];
    c_H[56] = angle_norm;
    angle_norm = cfg[12] + cfg[13] * c_norm(velocity_at_dvl);
    angle_norm *= angle_norm;
    for (i = 0; i < 9; i++) {
      Cbn[i] = angle_norm * (double)iv[i];
    }
    for (c_i = 0; c_i < 3; c_i++) {
      for (i = 0; i < 19; i++) {
        d_K[i + 19 * c_i] = c_H[c_i + 3 * i];
      }
    }
    memset(&d_H[0], 0, 57U * sizeof(double));
    for (i = 0; i < 19; i++) {
      angle_norm = d_H[3 * i];
      F_tmp = 3 * i + 1;
      Cnb_tmp = 3 * i + 2;
      for (b_i = 0; b_i < 19; b_i++) {
        S = covariance[b_i + 19 * i];
        angle_norm += c_H[3 * b_i] * S;
        d_H[F_tmp] += c_H[3 * b_i + 1] * S;
        d_H[Cnb_tmp] += c_H[3 * b_i + 2] * S;
      }
      d_H[3 * i] = angle_norm;
    }
    for (b_i = 0; b_i < 3; b_i++) {
      for (c_i = 0; c_i < 3; c_i++) {
        angle_norm = 0.0;
        for (i = 0; i < 19; i++) {
          angle_norm += d_H[b_i + 3 * i] * d_K[i + 19 * c_i];
        }
        F_tmp = b_i + 3 * c_i;
        b_S[F_tmp] = angle_norm + Cbn[F_tmp];
      }
    }
    mldivide(b_S, accel_corrected, w_en_ned);
    delta_quaternion_idx_3 =
        (accel_corrected[0] * w_en_ned[0] + accel_corrected[1] * w_en_ned[1]) +
        accel_corrected[2] * w_en_ned[2];
    if (!rtIsInf(delta_quaternion_idx_3) && !rtIsNaN(delta_quaternion_idx_3) &&
        (delta_quaternion_idx_3 <= 11.34)) {
      b = true;
    } else {
      b = false;
    }
    memset(&delta_state[0], 0, 19U * sizeof(double));
    if (b) {
      memset(&d_H[0], 0, 57U * sizeof(double));
      for (i = 0; i < 3; i++) {
        for (b_i = 0; b_i < 19; b_i++) {
          angle_norm = d_K[b_i + 19 * i];
          for (c_i = 0; c_i < 19; c_i++) {
            F_tmp = c_i + 19 * i;
            d_H[F_tmp] += covariance[c_i + 19 * b_i] * angle_norm;
          }
        }
      }
      mrdiv(d_H, b_S, d_K);
      memset(&delta_state[0], 0, 19U * sizeof(double));
      for (c_i = 0; c_i < 3; c_i++) {
        angle_norm = accel_corrected[c_i];
        for (i = 0; i < 19; i++) {
          delta_state[i] += d_K[i + 19 * c_i] * angle_norm;
        }
      }
      b_eye(F);
      for (i = 0; i < 19; i++) {
        angle_norm = d_K[i];
        S = d_K[i + 19];
        Cbn_tmp = d_K[i + 38];
        for (c_i = 0; c_i < 19; c_i++) {
          F_tmp = i + 19 * c_i;
          A[F_tmp] =
              F[F_tmp] - ((angle_norm * c_H[3 * c_i] + S * c_H[3 * c_i + 1]) +
                          Cbn_tmp * c_H[3 * c_i + 2]);
        }
      }
      memset(&b_F[0], 0, 361U * sizeof(double));
      for (i = 0; i < 19; i++) {
        for (b_i = 0; b_i < 19; b_i++) {
          angle_norm = covariance[b_i + 19 * i];
          for (c_i = 0; c_i < 19; c_i++) {
            F_tmp = c_i + 19 * i;
            b_F[F_tmp] += A[c_i + 19 * b_i] * angle_norm;
          }
        }
      }
      memset(&d_H[0], 0, 57U * sizeof(double));
      for (i = 0; i < 3; i++) {
        for (b_i = 0; b_i < 3; b_i++) {
          angle_norm = Cbn[b_i + 3 * i];
          for (c_i = 0; c_i < 19; c_i++) {
            F_tmp = c_i + 19 * i;
            d_H[F_tmp] += d_K[c_i + 19 * b_i] * angle_norm;
          }
        }
      }
      memset(&covariance[0], 0, 361U * sizeof(double));
      for (i = 0; i < 19; i++) {
        for (b_i = 0; b_i < 19; b_i++) {
          angle_norm = A[i + 19 * b_i];
          for (c_i = 0; c_i < 19; c_i++) {
            F_tmp = c_i + 19 * i;
            covariance[F_tmp] += b_F[c_i + 19 * b_i] * angle_norm;
          }
        }
      }
      memset(&b_F[0], 0, 361U * sizeof(double));
      for (i = 0; i < 19; i++) {
        for (b_i = 0; b_i < 3; b_i++) {
          angle_norm = d_K[i + 19 * b_i];
          for (c_i = 0; c_i < 19; c_i++) {
            F_tmp = c_i + 19 * i;
            b_F[F_tmp] += d_H[c_i + 19 * b_i] * angle_norm;
          }
        }
      }
      for (i = 0; i < 361; i++) {
        covariance[i] += b_F[i];
      }
      for (i = 0; i < 19; i++) {
        for (c_i = 0; c_i < 19; c_i++) {
          F_tmp = c_i + 19 * i;
          b_F[F_tmp] = 0.5 * (covariance[F_tmp] + covariance[i + 19 * c_i]);
        }
      }
      memcpy(&covariance[0], &b_F[0], 361U * sizeof(double));
    }
    update_status[2] = b;
    nis[2] = delta_quaternion_idx_3;
    if (b) {
      inject_error(delta_state, covariance, position, velocity, quaternion,
                   accel_bias, gyro_bias, &scale, boresight);
    }
  }
  state[0] = position[0];
  state[3] = velocity[0];
  state[1] = position[1];
  state[4] = velocity[1];
  state[2] = position[2];
  state[5] = velocity[2];
  state[6] = quaternion[0];
  state[7] = quaternion[1];
  state[8] = quaternion[2];
  state[9] = quaternion[3];
  state[16] = scale;
  state[10] = accel_bias[0];
  state[13] = gyro_bias[0];
  state[17] = boresight[0];
  state[11] = accel_bias[1];
  state[14] = gyro_bias[1];
  state[18] = boresight[1];
  state[12] = accel_bias[2];
  state[15] = gyro_bias[2];
  state[19] = boresight[2];
}

/* End of code generation (esekf19_step.c) */
