/*
 * Trial License - for use to evaluate programs for possible purchase as
 * an end-user only.
 *
 * _coder_esekf19_app_mex.c
 *
 * Code generation for function 'esekf19_app'
 *
 */

/* Include files */
#include "_coder_esekf19_app_mex.h"
#include "_coder_esekf19_app_api.h"

/* Function Definitions */
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  mexAtExit(&esekf19_app_atexit);
  esekf19_app_initialize();
  unsafe_esekf19_app_mexFunction(nlhs, plhs, nrhs, prhs);
  esekf19_app_terminate();
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1L,
                           NULL, "windows-1252", true);
  return emlrtRootTLSGlobal;
}

void unsafe_esekf19_app_mexFunction(int32_T nlhs, mxArray *plhs[3L],
                                    int32_T nrhs, const mxArray *prhs[4L])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *outputs[3L];
  int32_T i;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 4L) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5L, 12L, 4L,
                        4L, 11L, "esekf19_app");
  }
  if (nlhs > 3L) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3L, 4L,
                        11L, "esekf19_app");
  }
  /* Call the function. */
  esekf19_app_api(prhs, nlhs, outputs);
  /* Copy over outputs to the caller. */
  if (nlhs < 1L) {
    i = 1L;
  } else {
    i = nlhs;
  }
  emlrtReturnArrays(i, &plhs[0L], &outputs[0L]);
}

/* End of code generation (_coder_esekf19_app_mex.c) */
