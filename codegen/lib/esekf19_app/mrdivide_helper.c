/*
 * Trial License - for use to evaluate programs for possible purchase as
 * an end-user only.
 *
 * mrdivide_helper.c
 *
 * Code generation for function 'mrdivide_helper'
 *
 */

/* Include files */
#include "mrdivide_helper.h"
#include "rt_nonfinite.h"
#include "xgetrf.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
void b_mrdiv(double A[133], const double B[49])
{
  double b_A[49];
  double temp;
  int ipiv[7];
  int b_kBcol;
  int i;
  int j;
  int jAcol;
  int jBcol;
  int k;
  int kBcol;
  memcpy(&b_A[0], &B[0], 49U * sizeof(double));
  xgetrf(b_A, ipiv);
  for (j = 0; j < 7; j++) {
    jBcol = 19 * j - 1;
    jAcol = 7 * j;
    for (k = 0; k < j; k++) {
      kBcol = 19 * k;
      temp = b_A[k + jAcol];
      if (temp != 0.0) {
        for (i = 0; i < 19; i++) {
          b_kBcol = (i + jBcol) + 1;
          A[b_kBcol] -= temp * A[i + kBcol];
        }
      }
    }
    temp = 1.0 / b_A[j + jAcol];
    for (k = 0; k < 19; k++) {
      jAcol = (k + jBcol) + 1;
      A[jAcol] *= temp;
    }
  }
  for (k = 6; k >= 0; k--) {
    jAcol = 19 * k - 1;
    kBcol = 7 * k - 1;
    for (i = k + 2; i < 8; i++) {
      b_kBcol = 19 * (i - 1);
      temp = b_A[i + kBcol];
      if (temp != 0.0) {
        for (j = 0; j < 19; j++) {
          jBcol = (j + jAcol) + 1;
          A[jBcol] -= temp * A[j + b_kBcol];
        }
      }
    }
  }
  for (k = 5; k >= 0; k--) {
    jAcol = ipiv[k];
    if (jAcol != k + 1) {
      for (i = 0; i < 19; i++) {
        kBcol = i + 19 * k;
        temp = A[kBcol];
        b_kBcol = i + 19 * (jAcol - 1);
        A[kBcol] = A[b_kBcol];
        A[b_kBcol] = temp;
      }
    }
  }
}

void mrdiv(const double A[57], const double B[9], double Y[57])
{
  double b_A[9];
  double a21;
  double maxval;
  int k;
  int r1;
  int r2;
  int r3;
  int rtemp;
  for (k = 0; k < 9; k++) {
    b_A[k] = B[k];
  }
  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = fabs(B[0]);
  a21 = fabs(B[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }
  if (fabs(B[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }
  b_A[r2] = B[r2] / B[r1];
  b_A[r3] /= b_A[r1];
  b_A[r2 + 3] -= b_A[r2] * b_A[r1 + 3];
  b_A[r3 + 3] -= b_A[r3] * b_A[r1 + 3];
  b_A[r2 + 6] -= b_A[r2] * b_A[r1 + 6];
  b_A[r3 + 6] -= b_A[r3] * b_A[r1 + 6];
  if (fabs(b_A[r3 + 3]) > fabs(b_A[r2 + 3])) {
    rtemp = r2;
    r2 = r3;
    r3 = rtemp;
  }
  b_A[r3 + 3] /= b_A[r2 + 3];
  b_A[r3 + 6] -= b_A[r3 + 3] * b_A[r2 + 6];
  for (k = 0; k < 19; k++) {
    int Y_tmp;
    int b_Y_tmp;
    rtemp = k + 19 * r1;
    Y[rtemp] = A[k] / b_A[r1];
    Y_tmp = k + 19 * r2;
    Y[Y_tmp] = A[k + 19] - Y[rtemp] * b_A[r1 + 3];
    b_Y_tmp = k + 19 * r3;
    Y[b_Y_tmp] = A[k + 38] - Y[rtemp] * b_A[r1 + 6];
    Y[Y_tmp] /= b_A[r2 + 3];
    Y[b_Y_tmp] -= Y[Y_tmp] * b_A[r2 + 6];
    Y[b_Y_tmp] /= b_A[r3 + 6];
    Y[Y_tmp] -= Y[b_Y_tmp] * b_A[r3 + 3];
    Y[rtemp] -= Y[b_Y_tmp] * b_A[r3];
    Y[rtemp] -= Y[Y_tmp] * b_A[r2];
  }
}

/* End of code generation (mrdivide_helper.c) */
