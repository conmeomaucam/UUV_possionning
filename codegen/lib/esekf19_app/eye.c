/*
 * Trial License - for use to evaluate programs for possible purchase as
 * an end-user only.
 *
 * eye.c
 *
 * Code generation for function 'eye'
 *
 */

/* Include files */
#include "eye.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
void b_eye(double b_I[361])
{
  int k;
  memset(&b_I[0], 0, 361U * sizeof(double));
  for (k = 0; k < 19; k++) {
    b_I[k + 19 * k] = 1.0;
  }
}

void eye(double b_I[9])
{
  int i;
  for (i = 0; i < 9; i++) {
    b_I[i] = 0.0;
  }
  b_I[0] = 1.0;
  b_I[4] = 1.0;
  b_I[8] = 1.0;
}

/* End of code generation (eye.c) */
