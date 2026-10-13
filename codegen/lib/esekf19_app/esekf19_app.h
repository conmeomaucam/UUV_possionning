/*
 * Trial License - for use to evaluate programs for possible purchase as
 * an end-user only.
 *
 * esekf19_app.h
 *
 * Code generation for function 'esekf19_app'
 *
 */

#ifndef ESEKF19_APP_H
#define ESEKF19_APP_H

/* Include files */
#include "esekf19_app_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void esekf19_app(double mode, const struct0_T *init_in,
                        const struct1_T *meas_in, const struct2_T *cfg_in,
                        struct3_T *state_out, double status[3], double nis[3]);

void esekf19_app_init(void);

#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (esekf19_app.h) */
