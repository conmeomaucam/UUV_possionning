/*
 * Trial License - for use to evaluate programs for possible purchase as
 * an end-user only.
 *
 * xgetrf.c
 *
 * Code generation for function 'xgetrf'
 *
 */

/* Include files */
#include "xgetrf.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
long xgetrf(double A[49], int ipiv[7])
{
  int b_info;
  int ijA;
  int j;
  int k;
  for (k = 0; k < 7; k++) {
    ipiv[k] = k + 1;
  }
  b_info = 0;
  for (j = 0; j < 6; j++) {
    double smax;
    int a;
    int b;
    int jA;
    int mmj;
    mmj = 5 - j;
    b = j << 3U;
    jA = 8 - j;
    a = 0;
    smax = fabs(A[b]);
    for (k = 2; k < jA; k++) {
      double s;
      s = fabs(A[(b + k) - 1]);
      if (s > smax) {
        a = k - 1;
        smax = s;
      }
    }
    if (A[b + a] != 0.0) {
      if (a != 0) {
        jA = j + a;
        ipiv[j] = jA + 1;
        for (k = 0; k < 7; k++) {
          int i;
          a = j + k * 7;
          smax = A[a];
          i = jA + k * 7;
          A[a] = A[i];
          A[i] = smax;
        }
      }
      jA = (b - j) + 7;
      for (k = b + 2; k <= jA; k++) {
        A[k - 1] /= A[b];
      }
    } else {
      b_info = j + 1;
    }
    jA = b;
    for (k = 0; k <= mmj; k++) {
      smax = A[(b + k * 7) + 7];
      if (smax != 0.0) {
        a = (jA - j) + 14;
        for (ijA = jA + 9; ijA <= a; ijA++) {
          A[ijA - 1] += A[((b + ijA) - jA) - 8] * -smax;
        }
      }
      jA += 7;
    }
  }
  if ((b_info == 0) && !(A[48] != 0.0)) {
    b_info = 7;
  }
  return b_info;
}

/* End of code generation (xgetrf.c) */
