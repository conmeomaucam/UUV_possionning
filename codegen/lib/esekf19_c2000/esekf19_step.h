/*
 * Trial License - for use to evaluate programs for possible purchase as
 * an end-user only.
 *
 * esekf19_step.h
 *
 * Code generation for function 'esekf19_step'
 *
 */

#ifndef ESEKF19_STEP_H
#define ESEKF19_STEP_H

/* Include files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void esekf19_step(double state[20], double covariance[361],
                         const double process_noise[361],
                         const double gyro_meas[3], const double accel_meas[3],
                         const double gnss_meas[7], double depth_meas,
                         const double dvl_meas[3],
                         const double measurement_flags[3],
                         const double cfg[18], double update_status[3],
                         double nis[3]);

#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (esekf19_step.h) */
