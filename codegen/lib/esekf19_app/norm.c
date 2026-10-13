/*
 * Trial License - for use to evaluate programs for possible purchase as
 * an end-user only.
 *
 * norm.c
 *
 * Code generation for function 'norm'
 *
 */

/* Include files */
#include "norm.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
double b_norm(const double b_x[4])
{
  double absxk;
  double scale;
  double t;
  double y;
  bool b;
  scale = 3.312168642111238E-170;
  absxk = fabs(b_x[0]);
  if (absxk > 3.312168642111238E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.312168642111238E-170;
    y = t * t;
  }
  absxk = fabs(b_x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  absxk = fabs(b_x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  absxk = fabs(b_x[3]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  y = scale * sqrt(y);
  b = rtIsNaN(y);
  if (b) {
    int k;
    k = 0;
    long exitg1;
    do {
      exitg1 = 0L;
      if (k < 4) {
        if (rtIsNaN(b_x[k])) {
          exitg1 = 1L;
        } else {
          k++;
        }
      } else {
        y = rtInf;
        exitg1 = 1L;
      }
    } while (exitg1 == 0L);
  }
  return y;
}

double c_norm(const double b_x[3])
{
  double absxk;
  double scale;
  double t;
  double y;
  bool b;
  scale = 3.312168642111238E-170;
  absxk = fabs(b_x[0]);
  if (absxk > 3.312168642111238E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.312168642111238E-170;
    y = t * t;
  }
  absxk = fabs(b_x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  absxk = fabs(b_x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  y = scale * sqrt(y);
  b = rtIsNaN(y);
  if (b) {
    int k;
    k = 0;
    long exitg1;
    do {
      exitg1 = 0L;
      if (k < 3) {
        if (rtIsNaN(b_x[k])) {
          exitg1 = 1L;
        } else {
          k++;
        }
      } else {
        y = rtInf;
        exitg1 = 1L;
      }
    } while (exitg1 == 0L);
  }
  return y;
}

/* End of code generation (norm.c) */
