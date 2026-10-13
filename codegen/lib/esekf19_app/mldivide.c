/*
 * Trial License - for use to evaluate programs for possible purchase as
 * an end-user only.
 *
 * mldivide.c
 *
 * Code generation for function 'mldivide'
 *
 */

/* Include files */
#include "mldivide.h"
#include "rt_nonfinite.h"
#include "xgetrf.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
void b_mldivide(const double A[49], double B[7])
{
  double b_A[49];
  double temp;
  int ipiv[7];
  int b_i;
  int i;
  int kAcol;
  memcpy(&b_A[0], &A[0], 49U * sizeof(double));
  xgetrf(b_A, ipiv);
  for (i = 0; i < 6; i++) {
    kAcol = ipiv[i];
    if (kAcol != i + 1) {
      temp = B[i];
      B[i] = B[kAcol - 1];
      B[kAcol - 1] = temp;
    }
  }
  for (i = 0; i < 7; i++) {
    kAcol = 7 * i;
    if (B[i] != 0.0) {
      for (b_i = i + 2; b_i < 8; b_i++) {
        B[b_i - 1] -= B[i] * b_A[(b_i + kAcol) - 1];
      }
    }
  }
  for (i = 6; i >= 0; i--) {
    kAcol = 7 * i;
    temp = B[i];
    if (temp != 0.0) {
      temp /= b_A[i + kAcol];
      B[i] = temp;
      for (b_i = 0; b_i < i; b_i++) {
        B[b_i] -= B[i] * b_A[b_i + kAcol];
      }
    }
  }
}

void mldivide(const double A[9], const double B[3], double Y[3])
{
  double b_A[9];
  double a21;
  double maxval;
  int i;
  int r1;
  int r2;
  int r3;
  for (i = 0; i < 9; i++) {
    b_A[i] = A[i];
  }
  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = fabs(A[0]);
  a21 = fabs(A[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }
  if (fabs(A[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }
  b_A[r2] = A[r2] / A[r1];
  b_A[r3] /= b_A[r1];
  b_A[r2 + 3] -= b_A[r2] * b_A[r1 + 3];
  b_A[r3 + 3] -= b_A[r3] * b_A[r1 + 3];
  b_A[r2 + 6] -= b_A[r2] * b_A[r1 + 6];
  b_A[r3 + 6] -= b_A[r3] * b_A[r1 + 6];
  if (fabs(b_A[r3 + 3]) > fabs(b_A[r2 + 3])) {
    int rtemp;
    rtemp = r2;
    r2 = r3;
    r3 = rtemp;
  }
  b_A[r3 + 3] /= b_A[r2 + 3];
  b_A[r3 + 6] -= b_A[r3 + 3] * b_A[r2 + 6];
  Y[1] = B[r2] - B[r1] * b_A[r2];
  Y[2] = (B[r3] - B[r1] * b_A[r3]) - Y[1] * b_A[r3 + 3];
  Y[2] /= b_A[r3 + 6];
  Y[0] = B[r1] - Y[2] * b_A[r1 + 6];
  Y[1] -= Y[2] * b_A[r2 + 6];
  Y[1] /= b_A[r2 + 3];
  Y[0] -= Y[1] * b_A[r1 + 3];
  Y[0] /= b_A[r1];
}

/* End of code generation (mldivide.c) */
