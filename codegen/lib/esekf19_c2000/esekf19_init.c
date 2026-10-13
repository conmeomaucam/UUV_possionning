/*
 * Trial License - for use to evaluate programs for possible purchase as
 * an end-user only.
 *
 * esekf19_init.c
 *
 * Code generation for function 'esekf19_init'
 *
 */

/* Include files */
#include "esekf19_init.h"
#include "diag.h"
#include "esekf19_c2000_data.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
void esekf19_init(const double initial_rpy[3], double gravity,
                  const double noise_cfg[8], double x[20], double P[361],
                  double Q[361])
{
  double dv[19];
  double b_x_tmp;
  double c_x_tmp;
  double cp;
  double cr;
  double cy;
  double sp;
  double sr;
  double sy;
  double x_tmp;
  int i;
  /* ESEKF19_INIT Initialize fixed-size nominal state and error covariance. */
  /*  Nominal state x (20x1): p[1:3], v[4:6], q_wxyz[7:10], ba[11:13], */
  /*  bg[14:16], DVL scale[17], boresight alpha/beta/gamma[18:20]. */
  /*  Error state and covariance are fixed at 19 states. */
  /*  noise_cfg (8x1): [acc_vrw; gyro_arw; acc_bias_sigma; acc_bias_tau; */
  /*                    gyro_bias_rw; scale_rw; boresight_rw; dt]. */
  sy = initial_rpy[0] / 2.0;
  cr = cos(sy);
  sr = sin(sy);
  sy = initial_rpy[1] / 2.0;
  cp = cos(sy);
  sp = sin(sy);
  sy = initial_rpy[2] / 2.0;
  cy = cos(sy);
  sy = sin(sy);
  memset(&x[0], 0, 20U * sizeof(double));
  x_tmp = cy * cp;
  b_x_tmp = sy * sp;
  x[6] = x_tmp * cr + b_x_tmp * sr;
  x[7] = x_tmp * sr - b_x_tmp * cr;
  c_x_tmp = sy * cp;
  sy = cy * sp;
  x[8] = sy * cr + c_x_tmp * sr;
  x[9] = c_x_tmp * cr - sy * sr;
  sy = 0.0005 * gravity;
  sy *= sy;
  dv[15] = 0.0004;
  dv[0] = 0.0004;
  dv[3] = 0.0025000000000000005;
  dv[6] = 7.615435494667714E-5;
  dv[9] = sy;
  dv[12] = 5.876107634774471E-12;
  dv[16] = 0.0006853891945200943;
  dv[1] = 0.0004;
  dv[4] = 0.0025000000000000005;
  dv[7] = 7.615435494667714E-5;
  dv[10] = sy;
  dv[13] = 5.876107634774471E-12;
  dv[17] = 0.0006853891945200943;
  dv[2] = 0.0004;
  dv[5] = 0.0025000000000000005;
  dv[8] = 7.615435494667714E-5;
  dv[11] = sy;
  dv[14] = 5.876107634774471E-12;
  dv[18] = 0.0006853891945200943;
  diag(dv, P);
  memset(&Q[0], 0, 361U * sizeof(double));
  sy = noise_cfg[0] * noise_cfg[0] * noise_cfg[7];
  c_x_tmp = noise_cfg[1] * noise_cfg[1] * noise_cfg[7];
  x_tmp = noise_cfg[2] * noise_cfg[2] * (2.0 * noise_cfg[7] / noise_cfg[3]);
  b_x_tmp = noise_cfg[4] * noise_cfg[4] * noise_cfg[7];
  Q[300] = noise_cfg[5] * noise_cfg[5] * noise_cfg[7];
  cp = noise_cfg[6] * noise_cfg[6] * noise_cfg[7];
  for (i = 0; i < 3; i++) {
    int Q_tmp;
    int b_Q_tmp;
    int c_Q_tmp;
    int d_Q_tmp;
    int e_Q_tmp;
    int i1;
    i1 = iv[3 * i];
    Q_tmp = 19 * (i + 3);
    Q[Q_tmp + 3] = sy * (double)i1;
    b_Q_tmp = 19 * (i + 6);
    Q[b_Q_tmp + 6] = c_x_tmp * (double)i1;
    c_Q_tmp = 19 * (i + 9);
    Q[c_Q_tmp + 9] = x_tmp * (double)i1;
    d_Q_tmp = 19 * (i + 12);
    Q[d_Q_tmp + 12] = b_x_tmp * (double)i1;
    e_Q_tmp = 19 * (i + 16);
    Q[e_Q_tmp + 16] = cp * (double)i1;
    i1 = iv[3 * i + 1];
    Q[Q_tmp + 4] = sy * (double)i1;
    Q[b_Q_tmp + 7] = c_x_tmp * (double)i1;
    Q[c_Q_tmp + 10] = x_tmp * (double)i1;
    Q[d_Q_tmp + 13] = b_x_tmp * (double)i1;
    Q[e_Q_tmp + 17] = cp * (double)i1;
    i1 = iv[3 * i + 2];
    Q[Q_tmp + 5] = sy * (double)i1;
    Q[b_Q_tmp + 8] = c_x_tmp * (double)i1;
    Q[c_Q_tmp + 11] = x_tmp * (double)i1;
    Q[d_Q_tmp + 14] = b_x_tmp * (double)i1;
    Q[e_Q_tmp + 18] = cp * (double)i1;
  }
}

/* End of code generation (esekf19_init.c) */
