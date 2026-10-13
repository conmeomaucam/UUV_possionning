/*
 * Trial License - for use to evaluate programs for possible purchase as
 * an end-user only.
 *
 * atan2.c
 *
 * Code generation for function 'atan2'
 *
 */

/* Include files */
#include "atan2.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
double b_atan2(double y, double b_x)
{
  double r;
  if (rtIsNaN(y) || rtIsNaN(b_x)) {
    r = rtNaN;
  } else if (rtIsInf(y) && rtIsInf(b_x)) {
    int i;
    int i1;
    if (y > 0.0) {
      i = 1;
    } else {
      i = -1;
    }
    if (b_x > 0.0) {
      i1 = 1;
    } else {
      i1 = -1;
    }
    r = atan2(i, i1);
  } else if (b_x == 0.0) {
    if (y > 0.0) {
      r = RT_PI / 2.0;
    } else if (y < 0.0) {
      r = -(RT_PI / 2.0);
    } else {
      r = 0.0;
    }
  } else {
    r = atan2(y, b_x);
  }
  return r;
}

/* End of code generation (atan2.c) */
