/*
 * Trial License - for use to evaluate programs for possible purchase as
 * an end-user only.
 *
 * esekf19_app.c
 *
 * Code generation for function 'esekf19_app'
 *
 */

/* Include files */
#include "esekf19_app.h"
#include "atan2.h"
#include "diag.h"
#include "esekf19_app_data.h"
#include "esekf19_app_initialize.h"
#include "esekf19_app_types.h"
#include "esekf19_step.h"
#include "eye.h"
#include "mldivide.h"
#include "mrdivide_helper.h"
#include "norm.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Variable Definitions */
static double x[20];

static double P[361];

static double Q[361];

static double cfg_raw[18];

/* Function Definitions */
void esekf19_app(double mode, const struct0_T *init_in,
                 const struct1_T *meas_in, const struct2_T *cfg_in,
                 struct3_T *state_out, double status[3], double nis[3])
{
  static const int c_H[133] = {
      1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
      0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  static const int iv1[133] = {
      1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  static const int b_H[19] = {0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
                              0, 0, 0, 0, 0, 0, 0, 0, 0};
  static const int b_iv[19] = {0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0, 0, 0, 0};
  static const int a[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  static const int b[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  static const int b_a[9] = {-1, 0, 0, 0, -1, 0, 0, 0, -1};
  double F[361];
  double b_F[361];
  double b_K[133];
  double c_K[133];
  double d_H[57];
  double e_H[57];
  double H[19];
  double Cnb[9];
  double residual[7];
  double velocity[3];
  double velocity_at_dvl[3];
  double w_en_ned[3];
  double w_nb_body[3];
  double scale;
  int b_i;
  int c_i;
  int i;
  if (!isInitialized_esekf19_app) {
    esekf19_app_initialize();
  }
  /*  ESEKF19_APP Giao dien Wrapper cap cao duoc dong goi bang Struct. */
  /*  - mode: 0 = Khoi tao (Init), 1 = Chay buoc loc (Step) */
  /*  - state_out: Struct chua vi tri, van toc, quaternion, bias, boresight ro
   * rang */
  /*  Khai bao bo nho tinh (static) luu tru trang thai bo loc giua cac chu ky */
  status[0] = 0.0;
  nis[0] = 0.0;
  status[1] = 0.0;
  nis[1] = 0.0;
  status[2] = 0.0;
  nis[2] = 0.0;
  /*  1. CHE DO KHOI TAO (MODE = 0) */
  if (mode == 0.0) {
    double S;
    double b_x_tmp;
    double cp;
    double cr;
    double cy;
    double sp;
    double sr;
    double sy;
    double x_tmp;
    /*  Goi truc tiep ham esekf19_init goc */
    /* ESEKF19_INIT Initialize fixed-size nominal state and error covariance. */
    /*  Nominal state x (20x1): p[1:3], v[4:6], q_wxyz[7:10], ba[11:13], */
    /*  bg[14:16], DVL scale[17], boresight alpha/beta/gamma[18:20]. */
    /*  Error state and covariance are fixed at 19 states. */
    /*  noise_cfg (8x1): [acc_vrw; gyro_arw; acc_bias_sigma; acc_bias_tau; */
    /*                    gyro_bias_rw; scale_rw; boresight_rw; dt]. */
    sy = init_in->initial_rpy[0] / 2.0;
    cr = cos(sy);
    sr = sin(sy);
    sy = init_in->initial_rpy[1] / 2.0;
    cp = cos(sy);
    sp = sin(sy);
    sy = init_in->initial_rpy[2] / 2.0;
    cy = cos(sy);
    sy = sin(sy);
    memset(&x[0], 0, 20U * sizeof(double));
    x_tmp = cy * cp;
    b_x_tmp = sy * sp;
    x[6] = x_tmp * cr + b_x_tmp * sr;
    x[7] = x_tmp * sr - b_x_tmp * cr;
    S = sy * cp;
    sy = cy * sp;
    x[8] = sy * cr + S * sr;
    x[9] = S * cr - sy * sr;
    sy = 0.0005 * init_in->gravity;
    sy *= sy;
    H[15] = 0.0004;
    H[0] = 0.0004;
    H[3] = 0.0025000000000000005;
    H[6] = 7.615435494667714E-5;
    H[9] = sy;
    H[12] = 5.876107634774471E-12;
    H[16] = 0.0006853891945200943;
    H[1] = 0.0004;
    H[4] = 0.0025000000000000005;
    H[7] = 7.615435494667714E-5;
    H[10] = sy;
    H[13] = 5.876107634774471E-12;
    H[17] = 0.0006853891945200943;
    H[2] = 0.0004;
    H[5] = 0.0025000000000000005;
    H[8] = 7.615435494667714E-5;
    H[11] = sy;
    H[14] = 5.876107634774471E-12;
    H[18] = 0.0006853891945200943;
    diag(H, P);
    memset(&Q[0], 0, 361U * sizeof(double));
    sy = init_in->acc_vrw * init_in->acc_vrw * init_in->dt;
    S = init_in->gyro_arw * init_in->gyro_arw * init_in->dt;
    x_tmp = init_in->acc_bias_sigma * init_in->acc_bias_sigma *
            (2.0 * init_in->dt / init_in->acc_bias_tau);
    b_x_tmp = init_in->gyro_bias_rw * init_in->gyro_bias_rw * init_in->dt;
    Q[300] = init_in->scale_rw * init_in->scale_rw * init_in->dt;
    cp = init_in->boresight_rw * init_in->boresight_rw * init_in->dt;
    /*  Luu mang cfg phẳng cho cac chu ky step ve sau */
    cfg_raw[0] = cfg_in->dt;
    cfg_raw[1] = cfg_in->gravity;
    cfg_raw[5] = cfg_in->radius_n;
    cfg_raw[6] = cfg_in->radius_m;
    cfg_raw[7] = cfg_in->tan_latitude;
    cfg_raw[8] = cfg_in->height;
    for (i = 0; i < 3; i++) {
      int F_tmp;
      int Q_tmp;
      int b_Q_tmp;
      int c_Q_tmp;
      int d_Q_tmp;
      int e_Q_tmp;
      F_tmp = b[3 * i];
      Q_tmp = 19 * (i + 3);
      Q[Q_tmp + 3] = sy * (double)F_tmp;
      b_Q_tmp = 19 * (i + 6);
      Q[b_Q_tmp + 6] = S * (double)F_tmp;
      c_Q_tmp = 19 * (i + 9);
      Q[c_Q_tmp + 9] = x_tmp * (double)F_tmp;
      d_Q_tmp = 19 * (i + 12);
      Q[d_Q_tmp + 12] = b_x_tmp * (double)F_tmp;
      e_Q_tmp = 19 * (i + 16);
      Q[e_Q_tmp + 16] = cp * (double)F_tmp;
      F_tmp = b[3 * i + 1];
      Q[Q_tmp + 4] = sy * (double)F_tmp;
      Q[b_Q_tmp + 7] = S * (double)F_tmp;
      Q[c_Q_tmp + 10] = x_tmp * (double)F_tmp;
      Q[d_Q_tmp + 13] = b_x_tmp * (double)F_tmp;
      Q[e_Q_tmp + 17] = cp * (double)F_tmp;
      F_tmp = b[3 * i + 2];
      Q[Q_tmp + 5] = sy * (double)F_tmp;
      Q[b_Q_tmp + 8] = S * (double)F_tmp;
      Q[c_Q_tmp + 11] = x_tmp * (double)F_tmp;
      Q[d_Q_tmp + 14] = b_x_tmp * (double)F_tmp;
      Q[e_Q_tmp + 18] = cp * (double)F_tmp;
      cfg_raw[i + 2] = cfg_in->w_ie_ned[i];
      cfg_raw[i + 9] = cfg_in->lever_arm_body[i];
    }
    cfg_raw[12] = cfg_in->dvl_noise_floor;
    cfg_raw[13] = cfg_in->dvl_noise_slope;
    cfg_raw[14] = cfg_in->gps_pos_sigma;
    cfg_raw[15] = cfg_in->gps_vel_sigma;
    cfg_raw[16] = cfg_in->gps_yaw_sigma;
    cfg_raw[17] = cfg_in->depth_sigma;
    /*     %% 2. CHE DO CHAY BUOC LOC (MODE = 1) */
  } else {
    double A[361];
    double Cbn[9];
    double b_S[9];
    double dv[9];
    double quaternion[4];
    double accel_bias[3];
    double accel_corrected[3];
    double boresight[3];
    double gyro_bias[3];
    double gyro_corrected[3];
    double position[3];
    double S;
    double b_w_en_ned_tmp;
    double b_x_tmp;
    double cp;
    double cr;
    double cy;
    double delta_quaternion_idx_0;
    double delta_quaternion_idx_1;
    double delta_quaternion_idx_2;
    double sp;
    double sr;
    double sy;
    double w_en_ned_tmp;
    double x_tmp;
    int F_tmp;
    int Q_tmp;
    int b_Q_tmp;
    bool b_b;
    /*  Goi truc tiep ham esekf19_step goc */
    /* ESEKF19_STEP One fixed-size INS prediction and optional sensor updates.
     */
    /*  Code-generation entry point: no plotting, RNG, variable-size data or
     * I/O. */
    /*  state (20x1): p[1:3], v[4:6], quaternion wxyz[7:10], ba[11:13], */
    /*  bg[14:16], scale[17], boresight alpha/beta/gamma[18:20]. */
    /*  covariance/process_noise: fixed 19x19 error-state matrices. */
    /*  measurement_flags: [GNSS_valid; depth_valid; DVL_valid], fixed 3x1. */
    /*  gnss_meas (7x1): position NED, velocity NED, yaw (rad). */
    /*  cfg (18x1): dt, gravity, w_ie_NED[3], RN, RM, tan(latitude), height, */
    /*  lever_arm_body[3], DVL noise floor/slope, GPS pos/vel/yaw sigma, depth
     * sigma. */
    /*  viết code comment vào  */
    scale = x[16];
    accel_bias[0] = x[10];
    gyro_bias[0] = x[13];
    boresight[0] = x[17];
    accel_bias[1] = x[11];
    gyro_bias[1] = x[14];
    boresight[1] = x[18];
    accel_bias[2] = x[12];
    gyro_bias[2] = x[15];
    boresight[2] = x[19];
    /*  INS mechanization prediction in the local NED frame. */
    sy = b_norm(&x[6]);
    quaternion[0] = x[6] / sy;
    quaternion[1] = x[7] / sy;
    quaternion[2] = x[8] / sy;
    quaternion[3] = x[9] / sy;
    cp = quaternion[3] * quaternion[3];
    b_x_tmp = quaternion[2] * quaternion[2];
    Cbn[0] = 1.0 - 2.0 * (b_x_tmp + cp);
    sy = quaternion[1] * quaternion[2];
    x_tmp = quaternion[0] * quaternion[3];
    Cbn[3] = 2.0 * (sy - x_tmp);
    sp = quaternion[1] * quaternion[3];
    cy = quaternion[0] * quaternion[2];
    Cbn[6] = 2.0 * (sp + cy);
    Cbn[1] = 2.0 * (sy + x_tmp);
    S = quaternion[1] * quaternion[1];
    Cbn[4] = 1.0 - 2.0 * (S + cp);
    sy = quaternion[2] * quaternion[3];
    x_tmp = quaternion[0] * quaternion[1];
    Cbn[7] = 2.0 * (sy - x_tmp);
    Cbn[2] = 2.0 * (sp - cy);
    Cbn[5] = 2.0 * (sy + x_tmp);
    Cbn[8] = 1.0 - 2.0 * (S + b_x_tmp);
    w_en_ned_tmp = cfg_raw[5] + cfg_raw[8];
    w_en_ned[0] = x[4] / w_en_ned_tmp;
    b_w_en_ned_tmp = cfg_raw[6] + cfg_raw[8];
    w_en_ned[1] = -x[3] / b_w_en_ned_tmp;
    w_en_ned[2] = -x[4] * cfg_raw[7] / w_en_ned_tmp;
    accel_corrected[0] = meas_in->accel[0] - x[10];
    gyro_corrected[0] = meas_in->gyro[0] - x[13];
    sr = w_en_ned[0] + cfg_raw[2];
    accel_corrected[1] = meas_in->accel[1] - x[11];
    gyro_corrected[1] = meas_in->gyro[1] - x[14];
    cr = w_en_ned[1] + cfg_raw[3];
    accel_corrected[2] = meas_in->accel[2] - x[12];
    gyro_corrected[2] = meas_in->gyro[2] - x[15];
    delta_quaternion_idx_0 = w_en_ned[2] + cfg_raw[4];
    memset(&velocity_at_dvl[0], 0, 3U * sizeof(double));
    sy = velocity_at_dvl[0];
    x_tmp = velocity_at_dvl[1];
    b_x_tmp = velocity_at_dvl[2];
    for (i = 0; i < 3; i++) {
      S = Cbn[3 * i];
      cp = S * sr;
      sp = accel_corrected[i];
      sy += S * sp;
      S = Cbn[3 * i + 1];
      cp += S * cr;
      x_tmp += S * sp;
      S = Cbn[3 * i + 2];
      cp += S * delta_quaternion_idx_0;
      b_x_tmp += S * sp;
      w_nb_body[i] = gyro_corrected[i] - cp;
      w_en_ned[i] += 2.0 * cfg_raw[i + 2];
    }
    velocity[0] =
        x[3] + (sy - (w_en_ned[1] * x[5] - w_en_ned[2] * x[4])) * cfg_raw[0];
    velocity[1] =
        x[4] + (x_tmp - (w_en_ned[2] * x[3] - w_en_ned[0] * x[5])) * cfg_raw[0];
    velocity[2] = x[5] + ((cfg_raw[1] + b_x_tmp) -
                          (w_en_ned[0] * x[4] - w_en_ned[1] * x[3])) *
                             cfg_raw[0];
    position[0] = x[0] + velocity[0] * cfg_raw[0];
    sp = w_nb_body[0] * cfg_raw[0];
    w_en_ned[0] = sp;
    position[1] = x[1] + cfg_raw[0] * velocity[1];
    cy = cfg_raw[0] * w_nb_body[1];
    w_en_ned[1] = cy;
    position[2] = x[2] + cfg_raw[0] * velocity[2];
    cr = cfg_raw[0] * w_nb_body[2];
    w_en_ned[2] = cr;
    cp = c_norm(w_en_ned);
    sy = cp / 2.0;
    b_x_tmp = sin(sy);
    x_tmp = fmax(cp, 1.0E-12);
    delta_quaternion_idx_0 = cos(sy);
    delta_quaternion_idx_1 = b_x_tmp * sp / x_tmp;
    delta_quaternion_idx_2 = b_x_tmp * cy / x_tmp;
    sr = b_x_tmp * cr / x_tmp;
    quaternion[0] =
        ((delta_quaternion_idx_0 * x[6] - delta_quaternion_idx_1 * x[7]) -
         delta_quaternion_idx_2 * x[8]) -
        sr * x[9];
    quaternion[1] =
        ((delta_quaternion_idx_1 * x[6] + delta_quaternion_idx_0 * x[7]) +
         sr * x[8]) -
        delta_quaternion_idx_2 * x[9];
    quaternion[2] = ((delta_quaternion_idx_2 * x[6] - sr * x[7]) +
                     delta_quaternion_idx_0 * x[8]) +
                    delta_quaternion_idx_1 * x[9];
    quaternion[3] = ((sr * x[6] + delta_quaternion_idx_2 * x[7]) -
                     delta_quaternion_idx_1 * x[8]) +
                    delta_quaternion_idx_0 * x[9];
    sy = b_norm(quaternion);
    quaternion[0] /= sy;
    quaternion[1] /= sy;
    quaternion[2] /= sy;
    quaternion[3] /= sy;
    for (i = 0; i < 361; i++) {
      F[i] = iv[i];
    }
    sy = cfg_raw[0];
    for (i = 0; i < 3; i++) {
      F_tmp = 19 * (i + 3);
      F[F_tmp] = (double)a[3 * i] * sy;
      F[F_tmp + 1] = (double)a[3 * i + 1] * sy;
      F[F_tmp + 2] = (double)a[3 * i + 2] * sy;
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
      cp = Cnb[3 * b_i];
      F_tmp = 3 * b_i + 1;
      Q_tmp = 3 * b_i + 2;
      for (i = 0; i < 3; i++) {
        S = dv[i + 3 * b_i];
        cp += b_S[3 * i] * S;
        Cnb[F_tmp] += b_S[3 * i + 1] * S;
        Cnb[Q_tmp] += b_S[3 * i + 2] * S;
      }
      Cnb[3 * b_i] = cp;
      Q_tmp = 19 * (b_i + 6);
      F[Q_tmp + 3] = cp * cfg_raw[0];
      b_Q_tmp = 19 * (b_i + 9);
      F[b_Q_tmp + 3] = -Cbn[3 * b_i] * cfg_raw[0];
      F_tmp = 3 * b_i + 1;
      F[Q_tmp + 4] = Cnb[F_tmp] * cfg_raw[0];
      F[b_Q_tmp + 4] = -Cbn[F_tmp] * cfg_raw[0];
      F_tmp = 3 * b_i + 2;
      F[Q_tmp + 5] = Cnb[F_tmp] * cfg_raw[0];
      F[b_Q_tmp + 5] = -Cbn[F_tmp] * cfg_raw[0];
    }
    eye(b_S);
    sy = 0.0 * cfg_raw[0];
    nis[0] = sy;
    nis[1] = -w_nb_body[2] * cfg_raw[0];
    nis[2] = cy;
    w_en_ned[0] = cr;
    w_en_ned[1] = sy;
    w_en_ned[2] = -w_nb_body[0] * cfg_raw[0];
    status[0] = -w_nb_body[1] * cfg_raw[0];
    status[1] = sp;
    status[2] = sy;
    sy = cfg_raw[0];
    for (i = 0; i < 3; i++) {
      F_tmp = 19 * (i + 6);
      F[F_tmp + 6] = b_S[3 * i] - nis[i];
      Q_tmp = 3 * i + 1;
      F[F_tmp + 7] = b_S[Q_tmp] - w_en_ned[i];
      b_Q_tmp = 3 * i + 2;
      F[F_tmp + 8] = b_S[b_Q_tmp] - status[i];
      F_tmp = 19 * (i + 12);
      F[F_tmp + 6] = (double)b_a[3 * i] * sy;
      F[F_tmp + 7] = (double)b_a[Q_tmp] * sy;
      F[F_tmp + 8] = (double)b_a[b_Q_tmp] * sy;
    }
    memset(&b_F[0], 0, 361U * sizeof(double));
    for (i = 0; i < 19; i++) {
      for (c_i = 0; c_i < 19; c_i++) {
        sy = P[c_i + 19 * i];
        for (b_i = 0; b_i < 19; b_i++) {
          F_tmp = b_i + 19 * i;
          b_F[F_tmp] += F[b_i + 19 * c_i] * sy;
        }
      }
    }
    for (i = 0; i < 19; i++) {
      for (b_i = 0; b_i < 19; b_i++) {
        sy = 0.0;
        for (c_i = 0; c_i < 19; c_i++) {
          sy += b_F[i + 19 * c_i] * F[b_i + 19 * c_i];
        }
        F_tmp = i + 19 * b_i;
        P[F_tmp] = sy + Q[F_tmp];
      }
    }
    status[0] = 0.0;
    nis[0] = 0.0;
    status[1] = 0.0;
    nis[1] = 0.0;
    status[2] = 0.0;
    nis[2] = 0.0;
    if (meas_in->gnss_valid) {
      double R[49];
      double c_S[49];
      double c_a[7];
      sy = b_norm(quaternion);
      delta_quaternion_idx_2 = quaternion[2] / sy;
      sr = quaternion[3] / sy;
      sy =
          b_atan2(2.0 * (quaternion[1] / sy * delta_quaternion_idx_2 +
                         quaternion[0] / sy * sr),
                  1.0 - 2.0 * (delta_quaternion_idx_2 * delta_quaternion_idx_2 +
                               sr * sr));
      S = meas_in->gnss[6] - sy;
      sy = b_atan2(sin(S), cos(S));
      residual[6] = sy;
      sy = cfg_raw[14] * cfg_raw[14];
      S = cfg_raw[15] * cfg_raw[15];
      residual[0] = meas_in->gnss[0] - position[0];
      residual[3] = meas_in->gnss[3] - velocity[0];
      c_a[0] = sy;
      c_a[3] = S;
      residual[1] = meas_in->gnss[1] - position[1];
      residual[4] = meas_in->gnss[4] - velocity[1];
      c_a[1] = sy;
      c_a[4] = S;
      residual[2] = meas_in->gnss[2] - position[2];
      residual[5] = meas_in->gnss[5] - velocity[2];
      c_a[2] = sy;
      c_a[5] = S;
      c_a[6] = cfg_raw[16] * cfg_raw[16];
      b_diag(c_a, R);
      memset(&b_K[0], 0, 133U * sizeof(double));
      for (i = 0; i < 19; i++) {
        for (c_i = 0; c_i < 19; c_i++) {
          sy = P[c_i + 19 * i];
          for (b_i = 0; b_i < 7; b_i++) {
            F_tmp = b_i + 7 * i;
            b_K[F_tmp] += (double)c_H[b_i + 7 * c_i] * sy;
          }
        }
      }
      for (c_i = 0; c_i < 7; c_i++) {
        for (b_i = 0; b_i < 7; b_i++) {
          sy = 0.0;
          for (i = 0; i < 19; i++) {
            sy += b_K[c_i + 7 * i] * (double)iv1[i + 19 * b_i];
          }
          F_tmp = c_i + 7 * b_i;
          c_S[F_tmp] = sy + R[F_tmp];
        }
        c_a[c_i] = residual[c_i];
      }
      b_mldivide(c_S, c_a);
      x_tmp = 0.0;
      for (i = 0; i < 7; i++) {
        x_tmp += residual[i] * c_a[i];
      }
      if (!rtIsInf(x_tmp) && !rtIsNaN(x_tmp) && (x_tmp <= 18.48)) {
        b_b = true;
      } else {
        b_b = false;
      }
      memset(&H[0], 0, 19U * sizeof(double));
      if (b_b) {
        memset(&b_K[0], 0, 133U * sizeof(double));
        for (i = 0; i < 7; i++) {
          for (b_i = 0; b_i < 19; b_i++) {
            F_tmp = iv1[b_i + 19 * i];
            for (c_i = 0; c_i < 19; c_i++) {
              Q_tmp = c_i + 19 * i;
              b_K[Q_tmp] += P[c_i + 19 * b_i] * (double)F_tmp;
            }
          }
        }
        b_mrdiv(b_K, c_S);
        memset(&H[0], 0, 19U * sizeof(double));
        for (i = 0; i < 7; i++) {
          sy = residual[i];
          for (c_i = 0; c_i < 19; c_i++) {
            H[c_i] += b_K[c_i + 19 * i] * sy;
          }
        }
        b_eye(F);
        for (b_i = 0; b_i < 19; b_i++) {
          for (c_i = 0; c_i < 19; c_i++) {
            sy = 0.0;
            for (i = 0; i < 7; i++) {
              sy += b_K[b_i + 19 * i] * (double)c_H[i + 7 * c_i];
            }
            F_tmp = b_i + 19 * c_i;
            A[F_tmp] = F[F_tmp] - sy;
          }
        }
        memset(&b_F[0], 0, 361U * sizeof(double));
        for (i = 0; i < 19; i++) {
          for (b_i = 0; b_i < 19; b_i++) {
            sy = P[b_i + 19 * i];
            for (c_i = 0; c_i < 19; c_i++) {
              F_tmp = c_i + 19 * i;
              b_F[F_tmp] += A[c_i + 19 * b_i] * sy;
            }
          }
        }
        memset(&c_K[0], 0, 133U * sizeof(double));
        for (i = 0; i < 7; i++) {
          for (b_i = 0; b_i < 7; b_i++) {
            sy = R[b_i + 7 * i];
            for (c_i = 0; c_i < 19; c_i++) {
              F_tmp = c_i + 19 * i;
              c_K[F_tmp] += b_K[c_i + 19 * b_i] * sy;
            }
          }
        }
        memset(&F[0], 0, 361U * sizeof(double));
        for (i = 0; i < 19; i++) {
          for (b_i = 0; b_i < 19; b_i++) {
            sy = A[i + 19 * b_i];
            for (c_i = 0; c_i < 19; c_i++) {
              F_tmp = c_i + 19 * i;
              F[F_tmp] += b_F[c_i + 19 * b_i] * sy;
            }
          }
        }
        memset(&b_F[0], 0, 361U * sizeof(double));
        for (i = 0; i < 19; i++) {
          for (b_i = 0; b_i < 7; b_i++) {
            sy = b_K[i + 19 * b_i];
            for (c_i = 0; c_i < 19; c_i++) {
              F_tmp = c_i + 19 * i;
              b_F[F_tmp] += c_K[c_i + 19 * b_i] * sy;
            }
          }
        }
        for (i = 0; i < 361; i++) {
          P[i] = F[i] + b_F[i];
        }
        for (i = 0; i < 19; i++) {
          for (c_i = 0; c_i < 19; c_i++) {
            F_tmp = c_i + 19 * i;
            b_F[F_tmp] = 0.5 * (P[F_tmp] + P[i + 19 * c_i]);
          }
        }
        memcpy(&P[0], &b_F[0], 361U * sizeof(double));
      }
      status[0] = b_b;
      nis[0] = x_tmp;
      if (b_b) {
        scale = x[16];
        accel_bias[0] = x[10];
        gyro_bias[0] = x[13];
        boresight[0] = x[17];
        accel_bias[1] = x[11];
        gyro_bias[1] = x[14];
        boresight[1] = x[18];
        accel_bias[2] = x[12];
        gyro_bias[2] = x[15];
        boresight[2] = x[19];
        inject_error(H, P, position, velocity, quaternion, accel_bias,
                     gyro_bias, &scale, boresight);
      }
    }
    if (meas_in->depth_valid) {
      b_x_tmp = meas_in->depth - position[2];
      sp = cfg_raw[17] * cfg_raw[17];
      memset(&H[0], 0, 19U * sizeof(double));
      sy = 0.0;
      for (i = 0; i < 19; i++) {
        x_tmp = H[i];
        for (c_i = 0; c_i < 19; c_i++) {
          x_tmp += (double)b_H[c_i] * P[c_i + 19 * i];
        }
        H[i] = x_tmp;
        sy += x_tmp * (double)b_iv[i];
      }
      S = sy + sp;
      x_tmp = b_x_tmp * (b_x_tmp / S);
      if (!rtIsInf(x_tmp) && !rtIsNaN(x_tmp) && (x_tmp <= 6.63)) {
        b_b = true;
      } else {
        b_b = false;
      }
      memset(&H[0], 0, 19U * sizeof(double));
      if (b_b) {
        double K[19];
        for (i = 0; i < 19; i++) {
          cp = 0.0;
          for (c_i = 0; c_i < 19; c_i++) {
            cp += P[i + 19 * c_i] * (double)b_iv[c_i];
          }
          cp /= S;
          K[i] = cp;
          H[i] = cp * b_x_tmp;
        }
        b_eye(F);
        for (i = 0; i < 19; i++) {
          F_tmp = b_H[i];
          for (b_i = 0; b_i < 19; b_i++) {
            Q_tmp = b_i + 19 * i;
            A[Q_tmp] = F[Q_tmp] - K[b_i] * (double)F_tmp;
          }
        }
        memset(&b_F[0], 0, 361U * sizeof(double));
        for (i = 0; i < 19; i++) {
          for (b_i = 0; b_i < 19; b_i++) {
            sy = P[b_i + 19 * i];
            for (c_i = 0; c_i < 19; c_i++) {
              F_tmp = c_i + 19 * i;
              b_F[F_tmp] += A[c_i + 19 * b_i] * sy;
            }
          }
        }
        memset(&F[0], 0, 361U * sizeof(double));
        for (i = 0; i < 19; i++) {
          for (b_i = 0; b_i < 19; b_i++) {
            sy = A[i + 19 * b_i];
            for (c_i = 0; c_i < 19; c_i++) {
              F_tmp = c_i + 19 * i;
              F[F_tmp] += b_F[c_i + 19 * b_i] * sy;
            }
          }
        }
        for (c_i = 0; c_i < 19; c_i++) {
          for (i = 0; i < 19; i++) {
            b_F[i + 19 * c_i] = K[i] * sp * K[c_i];
          }
        }
        for (i = 0; i < 361; i++) {
          P[i] = F[i] + b_F[i];
        }
        for (i = 0; i < 19; i++) {
          for (c_i = 0; c_i < 19; c_i++) {
            F_tmp = c_i + 19 * i;
            b_F[F_tmp] = 0.5 * (P[F_tmp] + P[i + 19 * c_i]);
          }
        }
        memcpy(&P[0], &b_F[0], 361U * sizeof(double));
      }
      status[1] = b_b;
      nis[1] = x_tmp;
      if (b_b) {
        inject_error(H, P, position, velocity, quaternion, accel_bias,
                     gyro_bias, &scale, boresight);
      }
    }
    if (meas_in->dvl_valid) {
      double d_K[57];
      sy = b_norm(quaternion);
      delta_quaternion_idx_0 = quaternion[0] / sy;
      delta_quaternion_idx_1 = quaternion[1] / sy;
      delta_quaternion_idx_2 = quaternion[2] / sy;
      sr = quaternion[3] / sy;
      b_x_tmp = sr * sr;
      cp = delta_quaternion_idx_2 * delta_quaternion_idx_2;
      Cnb[0] = 1.0 - 2.0 * (cp + b_x_tmp);
      sy = delta_quaternion_idx_1 * delta_quaternion_idx_2;
      x_tmp = delta_quaternion_idx_0 * sr;
      Cnb[1] = 2.0 * (sy - x_tmp);
      sp = delta_quaternion_idx_1 * sr;
      cy = delta_quaternion_idx_0 * delta_quaternion_idx_2;
      Cnb[2] = 2.0 * (sp + cy);
      Cnb[3] = 2.0 * (sy + x_tmp);
      S = delta_quaternion_idx_1 * delta_quaternion_idx_1;
      Cnb[4] = 1.0 - 2.0 * (S + b_x_tmp);
      x_tmp = delta_quaternion_idx_2 * sr;
      sy = delta_quaternion_idx_0 * delta_quaternion_idx_1;
      Cnb[5] = 2.0 * (x_tmp - sy);
      Cnb[6] = 2.0 * (sp - cy);
      Cnb[7] = 2.0 * (x_tmp + sy);
      Cnb[8] = 1.0 - 2.0 * (S + cp);
      memset(&w_nb_body[0], 0, 3U * sizeof(double));
      sr = cfg_raw[2] + velocity[1] / w_en_ned_tmp;
      cr = cfg_raw[3] + -velocity[0] / b_w_en_ned_tmp;
      delta_quaternion_idx_0 =
          cfg_raw[4] + -velocity[1] * cfg_raw[7] / w_en_ned_tmp;
      S = w_nb_body[0];
      sp = w_nb_body[1];
      cy = w_nb_body[2];
      for (i = 0; i < 3; i++) {
        cp = velocity[i];
        S += Cnb[3 * i] * cp;
        sp += Cnb[3 * i + 1] * cp;
        cy += Cnb[3 * i + 2] * cp;
        w_en_ned[i] = gyro_corrected[i] - ((Cnb[i] * sr + Cnb[i + 3] * cr) +
                                           Cnb[i + 6] * delta_quaternion_idx_0);
      }
      velocity_at_dvl[0] =
          S + (w_en_ned[1] * cfg_raw[11] - w_en_ned[2] * cfg_raw[10]);
      velocity_at_dvl[1] =
          sp + (w_en_ned[2] * cfg_raw[9] - w_en_ned[0] * cfg_raw[11]);
      velocity_at_dvl[2] =
          cy + (w_en_ned[0] * cfg_raw[10] - w_en_ned[1] * cfg_raw[9]);
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
      sy = velocity_at_dvl[0];
      x_tmp = velocity_at_dvl[1];
      b_x_tmp = velocity_at_dvl[2];
      for (i = 0; i < 3; i++) {
        accel_corrected[i] =
            meas_in->dvl[i] -
            ((Cbn[i] * sy + Cbn[i + 3] * x_tmp) + Cbn[i + 6] * b_x_tmp);
      }
      memset(&d_H[0], 0, 57U * sizeof(double));
      dv[0] = 0.0;
      dv[3] = -cy;
      dv[6] = sp;
      dv[1] = cy;
      dv[4] = 0.0;
      dv[7] = -S;
      dv[2] = -sp;
      dv[5] = S;
      dv[8] = 0.0;
      for (b_i = 0; b_i < 3; b_i++) {
        F_tmp = 3 * (b_i + 3);
        d_H[F_tmp] = 0.0;
        d_H[F_tmp + 1] = 0.0;
        d_H[F_tmp + 2] = 0.0;
        for (i = 0; i < 3; i++) {
          sy = Cnb[i + 3 * b_i];
          d_H[F_tmp] += Cbn[3 * i] * sy;
          d_H[F_tmp + 1] += Cbn[3 * i + 1] * sy;
          d_H[F_tmp + 2] += Cbn[3 * i + 2] * sy;
          d_H[i + 3 * (b_i + 6)] = 0.0;
        }
        F_tmp = 3 * (b_i + 6);
        for (i = 0; i < 3; i++) {
          sy = dv[i + 3 * b_i];
          d_H[F_tmp] += Cbn[3 * i] * sy;
          d_H[F_tmp + 1] += Cbn[3 * i + 1] * sy;
          d_H[F_tmp + 2] += Cbn[3 * i + 2] * sy;
          d_H[i + 3 * (b_i + 12)] = 0.0;
        }
      }
      dv[0] = 0.0;
      dv[3] = -cfg_raw[11];
      dv[6] = cfg_raw[10];
      dv[1] = cfg_raw[11];
      dv[4] = 0.0;
      dv[7] = -cfg_raw[9];
      dv[2] = -cfg_raw[10];
      dv[5] = cfg_raw[9];
      dv[8] = 0.0;
      for (i = 0; i < 3; i++) {
        F_tmp = 3 * (i + 12);
        for (c_i = 0; c_i < 3; c_i++) {
          sy = dv[c_i + 3 * i];
          d_H[F_tmp] += Cbn[3 * c_i] * sy;
          d_H[F_tmp + 1] += Cbn[3 * c_i + 1] * sy;
          d_H[F_tmp + 2] += Cbn[3 * c_i + 2] * sy;
        }
        d_H[i + 45] = 0.0;
      }
      sy = d_H[45];
      S = d_H[46];
      x_tmp = d_H[47];
      for (i = 0; i < 3; i++) {
        b_x_tmp = velocity_at_dvl[i];
        sy += b_S[3 * i] * b_x_tmp;
        S += b_S[3 * i + 1] * b_x_tmp;
        x_tmp += b_S[3 * i + 2] * b_x_tmp;
      }
      d_H[47] = x_tmp;
      d_H[46] = S;
      d_H[45] = sy;
      sy = (scale + 1.0) * 0.0;
      d_H[48] = sy;
      d_H[51] = (scale + 1.0) * -velocity_at_dvl[2];
      d_H[54] = (scale + 1.0) * velocity_at_dvl[1];
      d_H[49] = (scale + 1.0) * velocity_at_dvl[2];
      d_H[52] = sy;
      d_H[55] = (scale + 1.0) * -velocity_at_dvl[0];
      d_H[50] = (scale + 1.0) * -velocity_at_dvl[1];
      d_H[53] = (scale + 1.0) * velocity_at_dvl[0];
      d_H[56] = sy;
      sy = cfg_raw[12] + cfg_raw[13] * c_norm(velocity_at_dvl);
      sy *= sy;
      for (i = 0; i < 9; i++) {
        Cbn[i] = sy * (double)a[i];
      }
      for (c_i = 0; c_i < 3; c_i++) {
        for (i = 0; i < 19; i++) {
          d_K[i + 19 * c_i] = d_H[c_i + 3 * i];
        }
      }
      memset(&e_H[0], 0, 57U * sizeof(double));
      for (i = 0; i < 19; i++) {
        sy = e_H[3 * i];
        F_tmp = 3 * i + 1;
        Q_tmp = 3 * i + 2;
        for (b_i = 0; b_i < 19; b_i++) {
          S = P[b_i + 19 * i];
          sy += d_H[3 * b_i] * S;
          e_H[F_tmp] += d_H[3 * b_i + 1] * S;
          e_H[Q_tmp] += d_H[3 * b_i + 2] * S;
        }
        e_H[3 * i] = sy;
      }
      for (b_i = 0; b_i < 3; b_i++) {
        for (c_i = 0; c_i < 3; c_i++) {
          sy = 0.0;
          for (i = 0; i < 19; i++) {
            sy += e_H[b_i + 3 * i] * d_K[i + 19 * c_i];
          }
          F_tmp = b_i + 3 * c_i;
          b_S[F_tmp] = sy + Cbn[F_tmp];
        }
      }
      mldivide(b_S, accel_corrected, w_en_ned);
      b_x_tmp = (accel_corrected[0] * w_en_ned[0] +
                 accel_corrected[1] * w_en_ned[1]) +
                accel_corrected[2] * w_en_ned[2];
      if (!rtIsInf(b_x_tmp) && !rtIsNaN(b_x_tmp) && (b_x_tmp <= 11.34)) {
        b_b = true;
      } else {
        b_b = false;
      }
      memset(&H[0], 0, 19U * sizeof(double));
      if (b_b) {
        memset(&e_H[0], 0, 57U * sizeof(double));
        for (i = 0; i < 3; i++) {
          for (b_i = 0; b_i < 19; b_i++) {
            sy = d_K[b_i + 19 * i];
            for (c_i = 0; c_i < 19; c_i++) {
              F_tmp = c_i + 19 * i;
              e_H[F_tmp] += P[c_i + 19 * b_i] * sy;
            }
          }
        }
        mrdiv(e_H, b_S, d_K);
        memset(&H[0], 0, 19U * sizeof(double));
        for (c_i = 0; c_i < 3; c_i++) {
          sy = accel_corrected[c_i];
          for (i = 0; i < 19; i++) {
            H[i] += d_K[i + 19 * c_i] * sy;
          }
        }
        b_eye(F);
        for (i = 0; i < 19; i++) {
          sy = d_K[i];
          S = d_K[i + 19];
          x_tmp = d_K[i + 38];
          for (c_i = 0; c_i < 19; c_i++) {
            F_tmp = i + 19 * c_i;
            A[F_tmp] = F[F_tmp] - ((sy * d_H[3 * c_i] + S * d_H[3 * c_i + 1]) +
                                   x_tmp * d_H[3 * c_i + 2]);
          }
        }
        memset(&b_F[0], 0, 361U * sizeof(double));
        for (i = 0; i < 19; i++) {
          for (b_i = 0; b_i < 19; b_i++) {
            sy = P[b_i + 19 * i];
            for (c_i = 0; c_i < 19; c_i++) {
              F_tmp = c_i + 19 * i;
              b_F[F_tmp] += A[c_i + 19 * b_i] * sy;
            }
          }
        }
        memset(&e_H[0], 0, 57U * sizeof(double));
        for (i = 0; i < 3; i++) {
          for (b_i = 0; b_i < 3; b_i++) {
            sy = Cbn[b_i + 3 * i];
            for (c_i = 0; c_i < 19; c_i++) {
              F_tmp = c_i + 19 * i;
              e_H[F_tmp] += d_K[c_i + 19 * b_i] * sy;
            }
          }
        }
        memset(&F[0], 0, 361U * sizeof(double));
        for (i = 0; i < 19; i++) {
          for (b_i = 0; b_i < 19; b_i++) {
            sy = A[i + 19 * b_i];
            for (c_i = 0; c_i < 19; c_i++) {
              F_tmp = c_i + 19 * i;
              F[F_tmp] += b_F[c_i + 19 * b_i] * sy;
            }
          }
        }
        memset(&b_F[0], 0, 361U * sizeof(double));
        for (i = 0; i < 19; i++) {
          for (b_i = 0; b_i < 3; b_i++) {
            sy = d_K[i + 19 * b_i];
            for (c_i = 0; c_i < 19; c_i++) {
              F_tmp = c_i + 19 * i;
              b_F[F_tmp] += e_H[c_i + 19 * b_i] * sy;
            }
          }
        }
        for (i = 0; i < 361; i++) {
          P[i] = F[i] + b_F[i];
        }
        for (i = 0; i < 19; i++) {
          for (c_i = 0; c_i < 19; c_i++) {
            F_tmp = c_i + 19 * i;
            b_F[F_tmp] = 0.5 * (P[F_tmp] + P[i + 19 * c_i]);
          }
        }
        memcpy(&P[0], &b_F[0], 361U * sizeof(double));
      }
      status[2] = b_b;
      nis[2] = b_x_tmp;
      if (b_b) {
        inject_error(H, P, position, velocity, quaternion, accel_bias,
                     gyro_bias, &scale, boresight);
      }
    }
    x[0] = position[0];
    x[3] = velocity[0];
    x[1] = position[1];
    x[4] = velocity[1];
    x[2] = position[2];
    x[5] = velocity[2];
    x[6] = quaternion[0];
    x[7] = quaternion[1];
    x[8] = quaternion[2];
    x[9] = quaternion[3];
    x[16] = scale;
    x[10] = accel_bias[0];
    x[13] = gyro_bias[0];
    x[17] = boresight[0];
    x[11] = accel_bias[1];
    x[14] = gyro_bias[1];
    x[18] = boresight[1];
    x[12] = accel_bias[2];
    x[15] = gyro_bias[2];
    x[19] = boresight[2];
  }
  /*  3. DONG GOI KET QUA RA STRUCT MINH BACH */
  state_out->pos_ned[0] = x[0];
  state_out->vel_ned[0] = x[3];
  state_out->pos_ned[1] = x[1];
  state_out->vel_ned[1] = x[4];
  state_out->pos_ned[2] = x[2];
  state_out->vel_ned[2] = x[5];
  state_out->quaternion[0] = x[6];
  state_out->quaternion[1] = x[7];
  state_out->quaternion[2] = x[8];
  state_out->quaternion[3] = x[9];
  state_out->dvl_scale = x[16];
  state_out->accel_bias[0] = x[10];
  state_out->gyro_bias[0] = x[13];
  state_out->dvl_boresight[0] = x[17];
  state_out->accel_bias[1] = x[11];
  state_out->gyro_bias[1] = x[14];
  state_out->dvl_boresight[1] = x[18];
  state_out->accel_bias[2] = x[12];
  state_out->gyro_bias[2] = x[15];
  state_out->dvl_boresight[2] = x[19];
}

void esekf19_app_init(void)
{
  memset(&x[0], 0, 20U * sizeof(double));
  memset(&P[0], 0, 361U * sizeof(double));
  memset(&Q[0], 0, 361U * sizeof(double));
  memset(&cfg_raw[0], 0, 18U * sizeof(double));
}

/* End of code generation (esekf19_app.c) */
