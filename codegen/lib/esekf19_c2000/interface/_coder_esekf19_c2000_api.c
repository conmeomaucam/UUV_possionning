/*
 * Trial License - for use to evaluate programs for possible purchase as
 * an end-user only.
 *
 * _coder_esekf19_c2000_api.c
 *
 * Code generation for function 'esekf19_init'
 *
 */

/* Include files */
#include "_coder_esekf19_c2000_api.h"
#include "_coder_esekf19_c2000_mex.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;

emlrtContext emlrtContextGlobal = {
    true,            /* bFirstTime */
    false,           /* bInitialized */
    131691UL,        /* fVersionInfo */
    NULL,            /* fErrorFunction */
    "esekf19_c2000", /* fFunctionName */
    NULL,            /* fRTCallStack */
    false,           /* bDebugMode */
    {2045744189UL, 2170104910UL, 2743257031UL, 4284093946UL}, /* fSigWrd */
    NULL                                                      /* fSigMem */
};

/* Function Declarations */
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[3L];

static const mxArray *b_emlrt_marshallOut(real_T u[361L]);

static real_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                 const char_T *identifier);

static void c_emlrt_marshallOut(real_T u[20L], const mxArray *y);

static real_T d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId);

static void d_emlrt_marshallOut(real_T u[361L], const mxArray *y);

static real_T (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                   const char_T *identifier))[8L];

static const mxArray *e_emlrt_marshallOut(real_T u[3L]);

static void emlrtExitTimeCleanupDtorFcn(void *r);

static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                 const char_T *identifier))[3L];

static const mxArray *emlrt_marshallOut(real_T u[20L]);

static real_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[8L];

static real_T (*g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                   const char_T *identifier))[20L];

static real_T (*h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[20L];

static real_T (*i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                   const char_T *identifier))[361L];

static real_T (*j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[361L];

static real_T (*k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                   const char_T *identifier))[7L];

static real_T (*l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[7L];

static real_T (*m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                   const char_T *identifier))[18L];

static real_T (*n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[18L];

static real_T (*o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[3L];

static real_T p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId);

static real_T (*q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[8L];

static real_T (*r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[20L];

static real_T (*s_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[361L];

static real_T (*t_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[7L];

static real_T (*u_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[18L];

/* Function Definitions */
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[3L]
{
  real_T(*y)[3L];
  y = o_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static const mxArray *b_emlrt_marshallOut(real_T u[361L])
{
  static const int32_T iv[2L] = {0L, 0L};
  static const int32_T iv1[2L] = {19L, 19L};
  const mxArray *m;
  const mxArray *y;
  void *existingData;
  y = NULL;
  m = emlrtCreateNumericArray(2L, (const void *)&iv[0L], mxDOUBLE_CLASS,
                              mxREAL);
  existingData = emlrtMxGetData((mxArray *)m);
  if (existingData != (void *)&u[0L]) {
    emlrtFreeMex(existingData);
  }
  emlrtMxSetData((mxArray *)m, &u[0L]);
  emlrtSetDimensions((mxArray *)m, &iv1[0L], 2L);
  emlrtAssign(&y, m);
  return y;
}

static real_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                 const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  real_T y;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = d_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId);
  emlrtDestroyArray(&nullptr);
  return y;
}

static void c_emlrt_marshallOut(real_T u[20L], const mxArray *y)
{
  static const int32_T i = 20L;
  void *existingData;
  existingData = emlrtMxGetData((mxArray *)y);
  if (existingData != (void *)&u[0L]) {
    emlrtFreeMex(existingData);
  }
  emlrtMxSetData((mxArray *)y, &u[0L]);
  emlrtSetDimensions((mxArray *)y, &i, 1L);
}

static real_T d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = p_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static void d_emlrt_marshallOut(real_T u[361L], const mxArray *y)
{
  static const int32_T iv[2L] = {19L, 19L};
  void *existingData;
  existingData = emlrtMxGetData((mxArray *)y);
  if (existingData != (void *)&u[0L]) {
    emlrtFreeMex(existingData);
  }
  emlrtMxSetData((mxArray *)y, &u[0L]);
  emlrtSetDimensions((mxArray *)y, &iv[0L], 2L);
}

static real_T (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                   const char_T *identifier))[8L]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[8L];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = f_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId);
  emlrtDestroyArray(&nullptr);
  return y;
}

static const mxArray *e_emlrt_marshallOut(real_T u[3L])
{
  static const int32_T i = 0L;
  static const int32_T i1 = 3L;
  const mxArray *m;
  const mxArray *y;
  void *existingData;
  y = NULL;
  m = emlrtCreateNumericArray(1L, (const void *)&i, mxDOUBLE_CLASS, mxREAL);
  existingData = emlrtMxGetData((mxArray *)m);
  if (existingData != (void *)&u[0L]) {
    emlrtFreeMex(existingData);
  }
  emlrtMxSetData((mxArray *)m, &u[0L]);
  emlrtSetDimensions((mxArray *)m, &i1, 1L);
  emlrtAssign(&y, m);
  return y;
}

static void emlrtExitTimeCleanupDtorFcn(void *r)
{
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                 const char_T *identifier))[3L]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[3L];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId);
  emlrtDestroyArray(&nullptr);
  return y;
}

static const mxArray *emlrt_marshallOut(real_T u[20L])
{
  static const int32_T i = 0L;
  static const int32_T i1 = 20L;
  const mxArray *m;
  const mxArray *y;
  void *existingData;
  y = NULL;
  m = emlrtCreateNumericArray(1L, (const void *)&i, mxDOUBLE_CLASS, mxREAL);
  existingData = emlrtMxGetData((mxArray *)m);
  if (existingData != (void *)&u[0L]) {
    emlrtFreeMex(existingData);
  }
  emlrtMxSetData((mxArray *)m, &u[0L]);
  emlrtSetDimensions((mxArray *)m, &i1, 1L);
  emlrtAssign(&y, m);
  return y;
}

static real_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[8L]
{
  real_T(*y)[8L];
  y = q_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                   const char_T *identifier))[20L]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[20L];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = h_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId);
  emlrtDestroyArray(&nullptr);
  return y;
}

static real_T (*h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[20L]
{
  real_T(*y)[20L];
  y = r_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                   const char_T *identifier))[361L]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[361L];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = j_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId);
  emlrtDestroyArray(&nullptr);
  return y;
}

static real_T (*j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[361L]
{
  real_T(*y)[361L];
  y = s_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                   const char_T *identifier))[7L]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[7L];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = l_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId);
  emlrtDestroyArray(&nullptr);
  return y;
}

static real_T (*l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[7L]
{
  real_T(*y)[7L];
  y = t_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                                   const char_T *identifier))[18L]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[18L];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = n_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId);
  emlrtDestroyArray(&nullptr);
  return y;
}

static real_T (*n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[18L]
{
  real_T(*y)[18L];
  y = u_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[3L]
{
  static const int32_T dims = 3L;
  real_T(*ret)[3L];
  int32_T i;
  boolean_T b = false;
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 1UL,
                            (const void *)&dims, &b, &i);
  ret = (real_T(*)[3L])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0L;
  real_T ret;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 0UL,
                          (const void *)&dims);
  ret = *(real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T (*q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[8L]
{
  static const int32_T dims = 8L;
  real_T(*ret)[8L];
  int32_T i;
  boolean_T b = false;
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 1UL,
                            (const void *)&dims, &b, &i);
  ret = (real_T(*)[8L])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T (*r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[20L]
{
  static const int32_T dims = 20L;
  real_T(*ret)[20L];
  int32_T i;
  boolean_T b = false;
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 1UL,
                            (const void *)&dims, &b, &i);
  ret = (real_T(*)[20L])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T (*s_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[361L]
{
  static const int32_T dims[2L] = {19L, 19L};
  real_T(*ret)[361L];
  int32_T iv[2L];
  boolean_T bv[2L] = {false, false};
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 2UL,
                            (const void *)&dims[0L], &bv[0L], &iv[0L]);
  ret = (real_T(*)[361L])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T (*t_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[7L]
{
  static const int32_T dims = 7L;
  real_T(*ret)[7L];
  int32_T i;
  boolean_T b = false;
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 1UL,
                            (const void *)&dims, &b, &i);
  ret = (real_T(*)[7L])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T (*u_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[18L]
{
  static const int32_T dims = 18L;
  real_T(*ret)[18L];
  int32_T i;
  boolean_T b = false;
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 1UL,
                            (const void *)&dims, &b, &i);
  ret = (real_T(*)[18L])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

void esekf19_c2000_atexit(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtPushHeapReferenceStackR2021a(
      &st, false, NULL, (void *)&emlrtExitTimeCleanupDtorFcn, NULL, NULL, NULL);
  emlrtEnterRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  esekf19_c2000_xil_terminate();
  esekf19_c2000_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

void esekf19_c2000_initialize(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0UL, NULL);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

void esekf19_c2000_terminate(void)
{
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

void esekf19_init_api(const mxArray *const prhs[3L], int32_T nlhs,
                      const mxArray *plhs[3L])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  real_T(*P)[361L];
  real_T(*Q)[361L];
  real_T(*x)[20L];
  real_T(*noise_cfg)[8L];
  real_T(*initial_rpy)[3L];
  real_T gravity;
  st.tls = emlrtRootTLSGlobal;
  x = (real_T(*)[20L])mxMalloc(sizeof(real_T[20L]));
  P = (real_T(*)[361L])mxMalloc(sizeof(real_T[361L]));
  Q = (real_T(*)[361L])mxMalloc(sizeof(real_T[361L]));
  /* Marshall function inputs */
  initial_rpy = emlrt_marshallIn(&st, emlrtAlias(prhs[0L]), "initial_rpy");
  gravity = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[1L]), "gravity");
  noise_cfg = e_emlrt_marshallIn(&st, emlrtAlias(prhs[2L]), "noise_cfg");
  /* Invoke the target function */
  esekf19_init(*initial_rpy, gravity, *noise_cfg, *x, *P, *Q);
  /* Marshall function outputs */
  plhs[0L] = emlrt_marshallOut(*x);
  if (nlhs > 1L) {
    plhs[1L] = b_emlrt_marshallOut(*P);
  }
  if (nlhs > 2L) {
    plhs[2L] = b_emlrt_marshallOut(*Q);
  }
}

void esekf19_step_api(const mxArray *const prhs[10L], int32_T nlhs,
                      const mxArray *plhs[4L])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *prhs_copy_idx_0;
  const mxArray *prhs_copy_idx_1;
  real_T(*covariance)[361L];
  real_T(*process_noise)[361L];
  real_T(*state)[20L];
  real_T(*cfg)[18L];
  real_T(*gnss_meas)[7L];
  real_T(*accel_meas)[3L];
  real_T(*dvl_meas)[3L];
  real_T(*gyro_meas)[3L];
  real_T(*measurement_flags)[3L];
  real_T(*nis)[3L];
  real_T(*update_status)[3L];
  real_T depth_meas;
  st.tls = emlrtRootTLSGlobal;
  update_status = (real_T(*)[3L])mxMalloc(sizeof(real_T[3L]));
  nis = (real_T(*)[3L])mxMalloc(sizeof(real_T[3L]));
  prhs_copy_idx_0 = emlrtProtectR2012b(prhs[0L], 0L, true, -1L);
  prhs_copy_idx_1 = emlrtProtectR2012b(prhs[1L], 1L, true, -1L);
  /* Marshall function inputs */
  state = g_emlrt_marshallIn(&st, emlrtAlias(prhs_copy_idx_0), "state");
  covariance =
      i_emlrt_marshallIn(&st, emlrtAlias(prhs_copy_idx_1), "covariance");
  process_noise =
      i_emlrt_marshallIn(&st, emlrtAlias(prhs[2L]), "process_noise");
  gyro_meas = emlrt_marshallIn(&st, emlrtAlias(prhs[3L]), "gyro_meas");
  accel_meas = emlrt_marshallIn(&st, emlrtAlias(prhs[4L]), "accel_meas");
  gnss_meas = k_emlrt_marshallIn(&st, emlrtAlias(prhs[5L]), "gnss_meas");
  depth_meas = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[6L]), "depth_meas");
  dvl_meas = emlrt_marshallIn(&st, emlrtAlias(prhs[7L]), "dvl_meas");
  measurement_flags =
      emlrt_marshallIn(&st, emlrtAlias(prhs[8L]), "measurement_flags");
  cfg = m_emlrt_marshallIn(&st, emlrtAlias(prhs[9L]), "cfg");
  /* Invoke the target function */
  esekf19_step(*state, *covariance, *process_noise, *gyro_meas, *accel_meas,
               *gnss_meas, depth_meas, *dvl_meas, *measurement_flags, *cfg,
               *update_status, *nis);
  /* Marshall function outputs */
  c_emlrt_marshallOut(*state, prhs_copy_idx_0);
  plhs[0L] = prhs_copy_idx_0;
  if (nlhs > 1L) {
    d_emlrt_marshallOut(*covariance, prhs_copy_idx_1);
    plhs[1L] = prhs_copy_idx_1;
  }
  if (nlhs > 2L) {
    plhs[2L] = e_emlrt_marshallOut(*update_status);
  }
  if (nlhs > 3L) {
    plhs[3L] = e_emlrt_marshallOut(*nis);
  }
}

/* End of code generation (_coder_esekf19_c2000_api.c) */
