/*
 * Trial License - for use to evaluate programs for possible purchase as
 * an end-user only.
 *
 * _coder_esekf19_c2000_mex.h
 *
 * Code generation for function 'esekf19_init'
 *
 */

#ifndef _CODER_ESEKF19_C2000_MEX_H
#define _CODER_ESEKF19_C2000_MEX_H

/* Include files */
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
MEXFUNCTION_LINKAGE void mexFunction(int32_T nlhs, mxArray *plhs[],
                                     int32_T nrhs, const mxArray *prhs[]);

emlrtCTX mexFunctionCreateRootTLS(void);

void unsafe_esekf19_init_mexFunction(int32_T nlhs, mxArray *plhs[3L],
                                     int32_T nrhs, const mxArray *prhs[3L]);

void unsafe_esekf19_step_mexFunction(int32_T nlhs, mxArray *plhs[4L],
                                     int32_T nrhs, const mxArray *prhs[10L]);

#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (_coder_esekf19_c2000_mex.h) */
