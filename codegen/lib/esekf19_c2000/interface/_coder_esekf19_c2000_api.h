/*
 * Trial License - for use to evaluate programs for possible purchase as
 * an end-user only.
 *
 * _coder_esekf19_c2000_api.h
 *
 * Code generation for function 'esekf19_init'
 *
 */

#ifndef _CODER_ESEKF19_C2000_API_H
#define _CODER_ESEKF19_C2000_API_H

/* Include files */
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"
#include <string.h>

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
void esekf19_c2000_atexit(void);

void esekf19_c2000_initialize(void);

void esekf19_c2000_terminate(void);

void esekf19_c2000_xil_shutdown(void);

void esekf19_c2000_xil_terminate(void);

void esekf19_init(const real_T initial_rpy[3L], real_T gravity,
                  const real_T noise_cfg[8L], real_T x[20L], real_T P[361L],
                  real_T Q[361L]);

void esekf19_init_api(const mxArray *const prhs[3L], int32_T nlhs,
                      const mxArray *plhs[3L]);

void esekf19_step(real_T state[20L], real_T covariance[361L],
                  const real_T process_noise[361L], const real_T gyro_meas[3L],
                  const real_T accel_meas[3L], const real_T gnss_meas[7L],
                  real_T depth_meas, const real_T dvl_meas[3L],
                  const real_T measurement_flags[3L], const real_T cfg[18L],
                  real_T update_status[3L], real_T nis[3L]);

void esekf19_step_api(const mxArray *const prhs[10L], int32_T nlhs,
                      const mxArray *plhs[4L]);

#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (_coder_esekf19_c2000_api.h) */
