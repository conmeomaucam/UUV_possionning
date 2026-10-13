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
void inject_error(const double delta_state[19], double covariance[361],
                  double position[3], double velocity[3], double quaternion[4],
                  double accel_bias[3], double gyro_bias[3], double *scale,
                  double boresight[3]);

#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (esekf19_step.h) */
