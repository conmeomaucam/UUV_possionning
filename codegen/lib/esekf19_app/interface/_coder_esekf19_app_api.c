/*
 * Trial License - for use to evaluate programs for possible purchase as
 * an end-user only.
 *
 * _coder_esekf19_app_api.c
 *
 * Code generation for function 'esekf19_app'
 *
 */

/* Include files */
#include "_coder_esekf19_app_api.h"
#include "_coder_esekf19_app_mex.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;

emlrtContext emlrtContextGlobal = {
    true,          /* bFirstTime */
    false,         /* bInitialized */
    131691UL,      /* fVersionInfo */
    NULL,          /* fErrorFunction */
    "esekf19_app", /* fFunctionName */
    NULL,          /* fRTCallStack */
    false,         /* bDebugMode */
    {2045744189UL, 2170104910UL, 2743257031UL, 4284093946UL}, /* fSigWrd */
    NULL                                                      /* fSigMem */
};

/* Function Declarations */
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId);

static const mxArray *b_emlrt_marshallOut(real_T u[3L]);

static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                               const char_T *identifier, struct0_T *y);

static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               struct0_T *y);

static void e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y[3L]);

static void emlrtExitTimeCleanupDtorFcn(void *r);

static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                               const char_T *identifier);

static const mxArray *emlrt_marshallOut(const struct3_T *u);

static void f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                               const char_T *identifier, struct1_T *y);

static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               struct1_T *y);

static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y[7L]);

static boolean_T i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId);

static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                               const char_T *identifier, struct2_T *y);

static void k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               struct2_T *y);

static real_T l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId);

static void m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[3L]);

static void n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[7L]);

static boolean_T o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId);

/* Function Definitions */
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = l_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static const mxArray *b_emlrt_marshallOut(real_T u[3L])
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

static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                               const char_T *identifier, struct0_T *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  d_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId, y);
  emlrtDestroyArray(&nullptr);
}

static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, struct0_T *y)
{
  static const int32_T dims = 0L;
  static const char_T *fieldNames[10L] = {"initial_rpy",    "gravity",
                                          "acc_vrw",        "gyro_arw",
                                          "acc_bias_sigma", "acc_bias_tau",
                                          "gyro_bias_rw",   "scale_rw",
                                          "boresight_rw",   "dt"};
  emlrtMsgIdentifier thisId;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtConstCTX)sp, parentId, u, 10L,
                         (const char_T **)&fieldNames[0L], 0UL,
                         (const void *)&dims);
  thisId.fIdentifier = "initial_rpy";
  e_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0L,
                                                    0L, "initial_rpy")),
                     &thisId, y->initial_rpy);
  thisId.fIdentifier = "gravity";
  y->gravity = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0L, 1L, "gravity")),
      &thisId);
  thisId.fIdentifier = "acc_vrw";
  y->acc_vrw = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0L, 2L, "acc_vrw")),
      &thisId);
  thisId.fIdentifier = "gyro_arw";
  y->gyro_arw = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0L, 3L, "gyro_arw")),
      &thisId);
  thisId.fIdentifier = "acc_bias_sigma";
  y->acc_bias_sigma =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtConstCTX)sp, u, 0L, 4L, "acc_bias_sigma")),
                         &thisId);
  thisId.fIdentifier = "acc_bias_tau";
  y->acc_bias_tau =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtConstCTX)sp, u, 0L, 5L, "acc_bias_tau")),
                         &thisId);
  thisId.fIdentifier = "gyro_bias_rw";
  y->gyro_bias_rw =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtConstCTX)sp, u, 0L, 6L, "gyro_bias_rw")),
                         &thisId);
  thisId.fIdentifier = "scale_rw";
  y->scale_rw = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0L, 7L, "scale_rw")),
      &thisId);
  thisId.fIdentifier = "boresight_rw";
  y->boresight_rw =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtConstCTX)sp, u, 0L, 8L, "boresight_rw")),
                         &thisId);
  thisId.fIdentifier = "dt";
  y->dt = b_emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0L, 9L, "dt")),
      &thisId);
  emlrtDestroyArray(&u);
}

static void e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[3L])
{
  m_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void emlrtExitTimeCleanupDtorFcn(void *r)
{
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                               const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  real_T y;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId);
  emlrtDestroyArray(&nullptr);
  return y;
}

static const mxArray *emlrt_marshallOut(const struct3_T *u)
{
  static const int32_T i = 3L;
  static const int32_T i1 = 3L;
  static const int32_T i2 = 4L;
  static const int32_T i3 = 3L;
  static const int32_T i4 = 3L;
  static const int32_T i5 = 3L;
  static const char_T *sv[7L] = {"pos_ned",      "vel_ned",   "quaternion",
                                 "accel_bias",   "gyro_bias", "dvl_scale",
                                 "dvl_boresight"};
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *d_y;
  const mxArray *e_y;
  const mxArray *f_y;
  const mxArray *g_y;
  const mxArray *h_y;
  const mxArray *m;
  const mxArray *y;
  real_T *pData;
  y = NULL;
  emlrtAssign(&y,
              emlrtCreateStructMatrix(1L, 1L, 7L, (const char_T **)&sv[0L]));
  b_y = NULL;
  m = emlrtCreateNumericArray(1L, (const void *)&i, mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  pData[0L] = u->pos_ned[0L];
  pData[1L] = u->pos_ned[1L];
  pData[2L] = u->pos_ned[2L];
  emlrtAssign(&b_y, m);
  emlrtSetFieldR2017b(y, 0L, "pos_ned", b_y, 0L);
  c_y = NULL;
  m = emlrtCreateNumericArray(1L, (const void *)&i1, mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  pData[0L] = u->vel_ned[0L];
  pData[1L] = u->vel_ned[1L];
  pData[2L] = u->vel_ned[2L];
  emlrtAssign(&c_y, m);
  emlrtSetFieldR2017b(y, 0L, "vel_ned", c_y, 1L);
  d_y = NULL;
  m = emlrtCreateNumericArray(1L, (const void *)&i2, mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  pData[0L] = u->quaternion[0L];
  pData[1L] = u->quaternion[1L];
  pData[2L] = u->quaternion[2L];
  pData[3L] = u->quaternion[3L];
  emlrtAssign(&d_y, m);
  emlrtSetFieldR2017b(y, 0L, "quaternion", d_y, 2L);
  e_y = NULL;
  m = emlrtCreateNumericArray(1L, (const void *)&i3, mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  pData[0L] = u->accel_bias[0L];
  pData[1L] = u->accel_bias[1L];
  pData[2L] = u->accel_bias[2L];
  emlrtAssign(&e_y, m);
  emlrtSetFieldR2017b(y, 0L, "accel_bias", e_y, 3L);
  f_y = NULL;
  m = emlrtCreateNumericArray(1L, (const void *)&i4, mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  pData[0L] = u->gyro_bias[0L];
  pData[1L] = u->gyro_bias[1L];
  pData[2L] = u->gyro_bias[2L];
  emlrtAssign(&f_y, m);
  emlrtSetFieldR2017b(y, 0L, "gyro_bias", f_y, 4L);
  g_y = NULL;
  m = emlrtCreateDoubleScalar(u->dvl_scale);
  emlrtAssign(&g_y, m);
  emlrtSetFieldR2017b(y, 0L, "dvl_scale", g_y, 5L);
  h_y = NULL;
  m = emlrtCreateNumericArray(1L, (const void *)&i5, mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  pData[0L] = u->dvl_boresight[0L];
  pData[1L] = u->dvl_boresight[1L];
  pData[2L] = u->dvl_boresight[2L];
  emlrtAssign(&h_y, m);
  emlrtSetFieldR2017b(y, 0L, "dvl_boresight", h_y, 6L);
  return y;
}

static void f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                               const char_T *identifier, struct1_T *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  g_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId, y);
  emlrtDestroyArray(&nullptr);
}

static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, struct1_T *y)
{
  static const int32_T dims = 0L;
  static const char_T *fieldNames[8L] = {
      "gyro", "accel",      "gnss",        "depth",
      "dvl",  "gnss_valid", "depth_valid", "dvl_valid"};
  emlrtMsgIdentifier thisId;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtConstCTX)sp, parentId, u, 8L,
                         (const char_T **)&fieldNames[0L], 0UL,
                         (const void *)&dims);
  thisId.fIdentifier = "gyro";
  e_emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0L, 0L, "gyro")),
      &thisId, y->gyro);
  thisId.fIdentifier = "accel";
  e_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0L, 1L, "accel")),
      &thisId, y->accel);
  thisId.fIdentifier = "gnss";
  h_emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0L, 2L, "gnss")),
      &thisId, y->gnss);
  thisId.fIdentifier = "depth";
  y->depth = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0L, 3L, "depth")),
      &thisId);
  thisId.fIdentifier = "dvl";
  e_emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0L, 4L, "dvl")),
      &thisId, y->dvl);
  thisId.fIdentifier = "gnss_valid";
  y->gnss_valid =
      i_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u,
                                                        0L, 5L, "gnss_valid")),
                         &thisId);
  thisId.fIdentifier = "depth_valid";
  y->depth_valid =
      i_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u,
                                                        0L, 6L, "depth_valid")),
                         &thisId);
  thisId.fIdentifier = "dvl_valid";
  y->dvl_valid =
      i_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u,
                                                        0L, 7L, "dvl_valid")),
                         &thisId);
  emlrtDestroyArray(&u);
}

static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[7L])
{
  n_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static boolean_T i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId)
{
  boolean_T y;
  y = o_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *nullptr,
                               const char_T *identifier, struct2_T *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  k_emlrt_marshallIn(sp, emlrtAlias(nullptr), &thisId, y);
  emlrtDestroyArray(&nullptr);
}

static void k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, struct2_T *y)
{
  static const int32_T dims = 0L;
  static const char_T *fieldNames[14L] = {"dt",
                                          "gravity",
                                          "w_ie_ned",
                                          "radius_n",
                                          "radius_m",
                                          "tan_latitude",
                                          "height",
                                          "lever_arm_body",
                                          "dvl_noise_floor",
                                          "dvl_noise_slope",
                                          "gps_pos_sigma",
                                          "gps_vel_sigma",
                                          "gps_yaw_sigma",
                                          "depth_sigma"};
  emlrtMsgIdentifier thisId;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtConstCTX)sp, parentId, u, 14L,
                         (const char_T **)&fieldNames[0L], 0UL,
                         (const void *)&dims);
  thisId.fIdentifier = "dt";
  y->dt = b_emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0L, 0L, "dt")),
      &thisId);
  thisId.fIdentifier = "gravity";
  y->gravity = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0L, 1L, "gravity")),
      &thisId);
  thisId.fIdentifier = "w_ie_ned";
  e_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0L, 2L, "w_ie_ned")),
      &thisId, y->w_ie_ned);
  thisId.fIdentifier = "radius_n";
  y->radius_n = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0L, 3L, "radius_n")),
      &thisId);
  thisId.fIdentifier = "radius_m";
  y->radius_m = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0L, 4L, "radius_m")),
      &thisId);
  thisId.fIdentifier = "tan_latitude";
  y->tan_latitude =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtConstCTX)sp, u, 0L, 5L, "tan_latitude")),
                         &thisId);
  thisId.fIdentifier = "height";
  y->height = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0L, 6L, "height")),
      &thisId);
  thisId.fIdentifier = "lever_arm_body";
  e_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)sp, u, 0L,
                                                    7L, "lever_arm_body")),
                     &thisId, y->lever_arm_body);
  thisId.fIdentifier = "dvl_noise_floor";
  y->dvl_noise_floor =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtConstCTX)sp, u, 0L, 8L, "dvl_noise_floor")),
                         &thisId);
  thisId.fIdentifier = "dvl_noise_slope";
  y->dvl_noise_slope =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtConstCTX)sp, u, 0L, 9L, "dvl_noise_slope")),
                         &thisId);
  thisId.fIdentifier = "gps_pos_sigma";
  y->gps_pos_sigma =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtConstCTX)sp, u, 0L, 10L, "gps_pos_sigma")),
                         &thisId);
  thisId.fIdentifier = "gps_vel_sigma";
  y->gps_vel_sigma =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtConstCTX)sp, u, 0L, 11L, "gps_vel_sigma")),
                         &thisId);
  thisId.fIdentifier = "gps_yaw_sigma";
  y->gps_yaw_sigma =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtConstCTX)sp, u, 0L, 12L, "gps_yaw_sigma")),
                         &thisId);
  thisId.fIdentifier = "depth_sigma";
  y->depth_sigma =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtConstCTX)sp, u, 0L, 13L, "depth_sigma")),
                         &thisId);
  emlrtDestroyArray(&u);
}

static real_T l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
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

static void m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[3L])
{
  static const int32_T dims = 3L;
  real_T(*r)[3L];
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 1UL,
                          (const void *)&dims);
  r = (real_T(*)[3L])emlrtMxGetData(src);
  ret[0L] = (*r)[0L];
  ret[1L] = (*r)[1L];
  ret[2L] = (*r)[2L];
  emlrtDestroyArray(&src);
}

static void n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[7L])
{
  static const int32_T dims = 7L;
  real_T(*r)[7L];
  int32_T i;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 1UL,
                          (const void *)&dims);
  r = (real_T(*)[7L])emlrtMxGetData(src);
  for (i = 0L; i < 7L; i++) {
    ret[i] = (*r)[i];
  }
  emlrtDestroyArray(&src);
}

static boolean_T o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0L;
  boolean_T ret;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "logical", false, 0UL,
                          (const void *)&dims);
  ret = *emlrtMxGetLogicals(src);
  emlrtDestroyArray(&src);
  return ret;
}

void esekf19_app_api(const mxArray *const prhs[4L], int32_T nlhs,
                     const mxArray *plhs[3L])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  struct0_T init_in;
  struct1_T meas_in;
  struct2_T cfg_in;
  struct3_T state_out;
  real_T(*nis)[3L];
  real_T(*status)[3L];
  real_T mode;
  st.tls = emlrtRootTLSGlobal;
  status = (real_T(*)[3L])mxMalloc(sizeof(real_T[3L]));
  nis = (real_T(*)[3L])mxMalloc(sizeof(real_T[3L]));
  /* Marshall function inputs */
  mode = emlrt_marshallIn(&st, emlrtAliasP(prhs[0L]), "mode");
  c_emlrt_marshallIn(&st, emlrtAliasP(prhs[1L]), "init_in", &init_in);
  f_emlrt_marshallIn(&st, emlrtAliasP(prhs[2L]), "meas_in", &meas_in);
  j_emlrt_marshallIn(&st, emlrtAliasP(prhs[3L]), "cfg_in", &cfg_in);
  /* Invoke the target function */
  esekf19_app(mode, &init_in, &meas_in, &cfg_in, &state_out, *status, *nis);
  /* Marshall function outputs */
  plhs[0L] = emlrt_marshallOut(&state_out);
  if (nlhs > 1L) {
    plhs[1L] = b_emlrt_marshallOut(*status);
  }
  if (nlhs > 2L) {
    plhs[2L] = b_emlrt_marshallOut(*nis);
  }
}

void esekf19_app_atexit(void)
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
  esekf19_app_xil_terminate();
  esekf19_app_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

void esekf19_app_initialize(void)
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

void esekf19_app_terminate(void)
{
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/* End of code generation (_coder_esekf19_app_api.c) */
