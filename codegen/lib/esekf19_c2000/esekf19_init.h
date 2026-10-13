/*
 * Trial License - for use to evaluate programs for possible purchase as
 * an end-user only.
 *
 * esekf19_init.h
 *
 * Code generation for function 'esekf19_init'
 *
 */

#ifndef ESEKF19_INIT_H
#define ESEKF19_INIT_H

/* Include files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void esekf19_init(const double initial_rpy[3], double gravity,
                         const double noise_cfg[8], double x[20], double P[361],
                         double Q[361]);

#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (esekf19_init.h) */
