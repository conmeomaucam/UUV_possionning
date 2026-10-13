/*
 * Trial License - for use to evaluate programs for possible purchase as
 * an end-user only.
 *
 * diag.c
 *
 * Code generation for function 'diag'
 *
 */

/* Include files */
#include "diag.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
void b_diag(const double v[7], double d[49])
{
  int j;
  memset(&d[0], 0, 49U * sizeof(double));
  for (j = 0; j < 7; j++) {
    d[j + 7 * j] = v[j];
  }
}

void diag(const double v[19], double d[361])
{
  int j;
  memset(&d[0], 0, 361U * sizeof(double));
  for (j = 0; j < 19; j++) {
    d[j + 19 * j] = v[j];
  }
}

/* End of code generation (diag.c) */
