/*
 * Trial License - for use to evaluate programs for possible purchase as
 * an end-user only.
 *
 * main.c
 *
 * Code generation for function 'main'
 *
 */

/*************************************************************************/
/* This automatically generated example C main file shows how to call    */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

/* Include files */
#include "main.h"
#include "esekf19_c2000_initialize.h"
#include "esekf19_c2000_terminate.h"
#include "esekf19_init.h"
#include "esekf19_step.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Declarations */
static void argInit_18x1_real_T(double result[18]);

static void argInit_19x19_real_T(double result[361]);

static void argInit_20x1_real_T(double result[20]);

static void argInit_3x1_real_T(double result[3]);

static void argInit_7x1_real_T(double result[7]);

static void argInit_8x1_real_T(double result[8]);

static double argInit_real_T(void);

/* Function Definitions */
static void argInit_18x1_real_T(double result[18])
{
  int idx0;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 18; idx0++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx0] = argInit_real_T();
  }
}

static void argInit_19x19_real_T(double result[361])
{
  int i;
  /* Loop over the array to initialize each element. */
  for (i = 0; i < 361; i++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[i] = argInit_real_T();
  }
}

static void argInit_20x1_real_T(double result[20])
{
  int idx0;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 20; idx0++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx0] = argInit_real_T();
  }
}

static void argInit_3x1_real_T(double result[3])
{
  int idx0;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 3; idx0++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx0] = argInit_real_T();
  }
}

static void argInit_7x1_real_T(double result[7])
{
  int idx0;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 7; idx0++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx0] = argInit_real_T();
  }
}

static void argInit_8x1_real_T(double result[8])
{
  int idx0;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 8; idx0++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx0] = argInit_real_T();
  }
}

static double argInit_real_T(void)
{
  return 0.0;
}

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;
  /* Initialize the application.
You do not need to do this more than one time. */
  esekf19_c2000_initialize();
  /* Invoke the entry-point functions.
You can call entry-point functions multiple times. */
  main_esekf19_init();
  main_esekf19_step();
  /* Terminate the application.
You do not need to do this more than one time. */
  esekf19_c2000_terminate();
  return 0;
}

void main_esekf19_init(void)
{
  double P[361];
  double Q[361];
  double x[20];
  double dv1[8];
  double dv[3];
  /* Initialize function 'esekf19_init' input arguments. */
  /* Initialize function input argument 'initial_rpy'. */
  /* Initialize function input argument 'noise_cfg'. */
  /* Call the entry-point 'esekf19_init'. */
  argInit_3x1_real_T(dv);
  argInit_8x1_real_T(dv1);
  esekf19_init(dv, argInit_real_T(), dv1, x, P, Q);
}

void main_esekf19_step(void)
{
  double covariance[361];
  double covariance_tmp[361];
  double state[20];
  double dv1[18];
  double dv[7];
  double gyro_meas_tmp[3];
  double nis[3];
  double update_status[3];
  /* Initialize function 'esekf19_step' input arguments. */
  /* Initialize function input argument 'state'. */
  /* Initialize function input argument 'covariance'. */
  argInit_19x19_real_T(covariance_tmp);
  /* Initialize function input argument 'process_noise'. */
  /* Initialize function input argument 'gyro_meas'. */
  argInit_3x1_real_T(gyro_meas_tmp);
  /* Initialize function input argument 'accel_meas'. */
  /* Initialize function input argument 'gnss_meas'. */
  /* Initialize function input argument 'dvl_meas'. */
  /* Initialize function input argument 'measurement_flags'. */
  /* Initialize function input argument 'cfg'. */
  /* Call the entry-point 'esekf19_step'. */
  argInit_20x1_real_T(state);
  memcpy(&covariance[0], &covariance_tmp[0], 361U * sizeof(double));
  argInit_7x1_real_T(dv);
  argInit_18x1_real_T(dv1);
  esekf19_step(state, covariance, covariance_tmp, gyro_meas_tmp, gyro_meas_tmp,
               dv, argInit_real_T(), gyro_meas_tmp, gyro_meas_tmp, dv1,
               update_status, nis);
}

/* End of code generation (main.c) */
