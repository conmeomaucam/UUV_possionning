/*
 * Trial License - for use to evaluate programs for possible purchase as
 * an end-user only.
 *
 * _coder_esekf19_c2000_mex.c
 *
 * Code generation for function 'esekf19_init'
 *
 */

/* Include files */
#include "_coder_esekf19_c2000_mex.h"
#include "_coder_esekf19_c2000_api.h"

/* Function Definitions */
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const char_T *entryPointTemplateNames[2L] = {"esekf19_init", "esekf19_step"};
  mexAtExit(&esekf19_c2000_atexit);
  esekf19_c2000_initialize();
  st.tls = emlrtRootTLSGlobal;
  switch (emlrtGetEntryPointIndexR2016a(
      &st, nrhs, &prhs[0L], (const char_T **)&entryPointTemplateNames[0L],
      2L)) {
  case 0L:
    unsafe_esekf19_init_mexFunction(nlhs, plhs, nrhs - 1L, &prhs[1L]);
    break;
  case 1L:
    unsafe_esekf19_step_mexFunction(nlhs, plhs, nrhs - 1L, &prhs[1L]);
    break;
  }
  esekf19_c2000_terminate();
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1L,
                           NULL, "windows-1252", true);
  return emlrtRootTLSGlobal;
}

void unsafe_esekf19_init_mexFunction(int32_T nlhs, mxArray *plhs[3L],
                                     int32_T nrhs, const mxArray *prhs[3L])
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
  if (nrhs != 3L) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5L, 12L, 3L,
                        4L, 12L, "esekf19_init");
  }
  if (nlhs > 3L) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3L, 4L,
                        12L, "esekf19_init");
  }
  /* Call the function. */
  esekf19_init_api(prhs, nlhs, outputs);
  /* Copy over outputs to the caller. */
  if (nlhs < 1L) {
    i = 1L;
  } else {
    i = nlhs;
  }
  emlrtReturnArrays(i, &plhs[0L], &outputs[0L]);
}

void unsafe_esekf19_step_mexFunction(int32_T nlhs, mxArray *plhs[4L],
                                     int32_T nrhs, const mxArray *prhs[10L])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *outputs[4L];
  int32_T i;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 10L) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5L, 12L, 10L,
                        4L, 12L, "esekf19_step");
  }
  if (nlhs > 4L) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3L, 4L,
                        12L, "esekf19_step");
  }
  /* Call the function. */
  esekf19_step_api(prhs, nlhs, outputs);
  /* Copy over outputs to the caller. */
  if (nlhs < 1L) {
    i = 1L;
  } else {
    i = nlhs;
  }
  emlrtReturnArrays(i, &plhs[0L], &outputs[0L]);
}

/* End of code generation (_coder_esekf19_c2000_mex.c) */
