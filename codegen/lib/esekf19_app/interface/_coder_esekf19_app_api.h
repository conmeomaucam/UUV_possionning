/*
 * Trial License - for use to evaluate programs for possible purchase as
 * an end-user only.
 *
 * _coder_esekf19_app_api.h
 *
 * Code generation for function 'esekf19_app'
 *
 */

#ifndef _CODER_ESEKF19_APP_API_H
#define _CODER_ESEKF19_APP_API_H

/* Include files */
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"
#include <string.h>

/* Type Definitions */
#ifndef typedef_struct0_T
#define typedef_struct0_T
typedef struct {
  real_T initial_rpy[3L];
  real_T gravity;
  real_T acc_vrw;
  real_T gyro_arw;
  real_T acc_bias_sigma;
  real_T acc_bias_tau;
  real_T gyro_bias_rw;
  real_T scale_rw;
  real_T boresight_rw;
  real_T dt;
} struct0_T;
#endif /* typedef_struct0_T */

#ifndef typedef_struct1_T
#define typedef_struct1_T
typedef struct {
  real_T gyro[3L];
  real_T accel[3L];
  real_T gnss[7L];
  real_T depth;
  real_T dvl[3L];
  boolean_T gnss_valid;
  boolean_T depth_valid;
  boolean_T dvl_valid;
} struct1_T;
#endif /* typedef_struct1_T */

#ifndef typedef_struct2_T
#define typedef_struct2_T
typedef struct {
  real_T dt;
  real_T gravity;
  real_T w_ie_ned[3L];
  real_T radius_n;
  real_T radius_m;
  real_T tan_latitude;
  real_T height;
  real_T lever_arm_body[3L];
  real_T dvl_noise_floor;
  real_T dvl_noise_slope;
  real_T gps_pos_sigma;
  real_T gps_vel_sigma;
  real_T gps_yaw_sigma;
  real_T depth_sigma;
} struct2_T;
#endif /* typedef_struct2_T */

#ifndef typedef_struct3_T
#define typedef_struct3_T
typedef struct {
  real_T pos_ned[3L];
  real_T vel_ned[3L];
  real_T quaternion[4L];
  real_T accel_bias[3L];
  real_T gyro_bias[3L];
  real_T dvl_scale;
  real_T dvl_boresight[3L];
} struct3_T;
#endif /* typedef_struct3_T */

/* Variable Declarations */

#ifdef __cplusplus
extern "C" {
#endif

extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void esekf19_app(real_T mode, const struct0_T *init_in,
                 const struct1_T *meas_in, const struct2_T *cfg_in,
                 struct3_T *state_out, real_T status[3L], real_T nis[3L]);

void esekf19_app_api(const mxArray *const prhs[4L], int32_T nlhs,
                     const mxArray *plhs[3L]);

void esekf19_app_atexit(void);

void esekf19_app_initialize(void);

void esekf19_app_terminate(void);

void esekf19_app_xil_shutdown(void);

void esekf19_app_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (_coder_esekf19_app_api.h) */
