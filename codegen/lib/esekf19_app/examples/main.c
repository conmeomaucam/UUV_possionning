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
#include "esekf19_app.h"
#include "esekf19_app_initialize.h"
#include "esekf19_app_terminate.h"
#include "esekf19_app_types.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static void argInit_3x1_real_T(double result[3]);

static void argInit_7x1_real_T(double result[7]);

static bool argInit_boolean_T(void);

static double argInit_real_T(void);

static void argInit_struct0_T(struct0_T *result);

static void argInit_struct1_T(struct1_T *result);

static void argInit_struct2_T(struct2_T *result);

/* Function Definitions */
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

static bool argInit_boolean_T(void)
{
  return false;
}

static double argInit_real_T(void)
{
  return 0.0;
}

static void argInit_struct0_T(struct0_T *result)
{
  double result_tmp;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result_tmp = argInit_real_T();
  argInit_3x1_real_T(result->initial_rpy);
  result->gravity = result_tmp;
  result->acc_vrw = result_tmp;
  result->gyro_arw = result_tmp;
  result->acc_bias_sigma = result_tmp;
  result->acc_bias_tau = result_tmp;
  result->gyro_bias_rw = result_tmp;
  result->scale_rw = result_tmp;
  result->boresight_rw = result_tmp;
  result->dt = result_tmp;
}

static void argInit_struct1_T(struct1_T *result)
{
  bool result_tmp;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  argInit_3x1_real_T(result->gyro);
  result_tmp = argInit_boolean_T();
  argInit_7x1_real_T(result->gnss);
  result->depth = argInit_real_T();
  result->gnss_valid = result_tmp;
  result->depth_valid = result_tmp;
  result->dvl_valid = result_tmp;
  result->accel[0] = result->gyro[0];
  result->dvl[0] = result->gyro[0];
  result->accel[1] = result->gyro[1];
  result->dvl[1] = result->gyro[1];
  result->accel[2] = result->gyro[2];
  result->dvl[2] = result->gyro[2];
}

static void argInit_struct2_T(struct2_T *result)
{
  double result_tmp;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result_tmp = argInit_real_T();
  argInit_3x1_real_T(result->w_ie_ned);
  result->dt = result_tmp;
  result->gravity = result_tmp;
  result->radius_n = result_tmp;
  result->radius_m = result_tmp;
  result->tan_latitude = result_tmp;
  result->height = result_tmp;
  result->dvl_noise_floor = result_tmp;
  result->dvl_noise_slope = result_tmp;
  result->gps_pos_sigma = result_tmp;
  result->gps_vel_sigma = result_tmp;
  result->gps_yaw_sigma = result_tmp;
  result->depth_sigma = result_tmp;
  result->lever_arm_body[0] = result->w_ie_ned[0];
  result->lever_arm_body[1] = result->w_ie_ned[1];
  result->lever_arm_body[2] = result->w_ie_ned[2];
}

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;
  /* Initialize the application.
You do not need to do this more than one time. */
  esekf19_app_initialize();
  /* Invoke the entry-point functions.
You can call entry-point functions multiple times. */
  main_esekf19_app();
  /* Terminate the application.
You do not need to do this more than one time. */
  esekf19_app_terminate();
  return 0;
}

void main_esekf19_app(void)
{
  struct0_T r;
  struct1_T r1;
  struct2_T r2;
  struct3_T state_out;
  double nis[3];
  double status[3];
  /* Initialize function 'esekf19_app' input arguments. */
  /* Initialize function input argument 'init_in'. */
  /* Initialize function input argument 'meas_in'. */
  /* Initialize function input argument 'cfg_in'. */
  /* Call the entry-point 'esekf19_app'. */
  argInit_struct0_T(&r);
  argInit_struct1_T(&r1);
  argInit_struct2_T(&r2);
  esekf19_app(argInit_real_T(), &r, &r1, &r2, &state_out, status, nis);
}

/* End of code generation (main.c) */
