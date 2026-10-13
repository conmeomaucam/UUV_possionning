/*
 * Trial License - for use to evaluate programs for possible purchase as
 * an end-user only.
 *
 * _coder_esekf19_app_mex.h
 *
 * Code generation for function 'esekf19_app'
 *
 */

#ifndef _CODER_ESEKF19_APP_MEX_H
#define _CODER_ESEKF19_APP_MEX_H

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

void unsafe_esekf19_app_mexFunction(int32_T nlhs, mxArray *plhs[3L],
                                    int32_T nrhs, const mxArray *prhs[4L]);

#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (_coder_esekf19_app_mex.h) */
