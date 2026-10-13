/*
 * Trial License - for use to evaluate programs for possible purchase as
 * an end-user only.
 *
 * esekf19_app_types.h
 *
 * Code generation for function 'esekf19_app'
 *
 */

#ifndef ESEKF19_APP_TYPES_H
#define ESEKF19_APP_TYPES_H

/* Include files */
#include "rtwtypes.h"

/* Type Definitions */
#ifndef typedef_struct0_T
#define typedef_struct0_T
typedef struct {
  double initial_rpy[3];
  double gravity;
  double acc_vrw;
  double gyro_arw;
  double acc_bias_sigma;
  double acc_bias_tau;
  double gyro_bias_rw;
  double scale_rw;
  double boresight_rw;
  double dt;
} struct0_T;
#endif /* typedef_struct0_T */

#ifndef typedef_struct1_T
#define typedef_struct1_T
typedef struct {
  double gyro[3];
  double accel[3];
  double gnss[7];
  double depth;
  double dvl[3];
  bool gnss_valid;
  bool depth_valid;
  bool dvl_valid;
} struct1_T;
#endif /* typedef_struct1_T */

#ifndef typedef_struct2_T
#define typedef_struct2_T
typedef struct {
  double dt;
  double gravity;
  double w_ie_ned[3];
  double radius_n;
  double radius_m;
  double tan_latitude;
  double height;
  double lever_arm_body[3];
  double dvl_noise_floor;
  double dvl_noise_slope;
  double gps_pos_sigma;
  double gps_vel_sigma;
  double gps_yaw_sigma;
  double depth_sigma;
} struct2_T;
#endif /* typedef_struct2_T */

#ifndef typedef_struct3_T
#define typedef_struct3_T
typedef struct {
  double pos_ned[3];
  double vel_ned[3];
  double quaternion[4];
  double accel_bias[3];
  double gyro_bias[3];
  double dvl_scale;
  double dvl_boresight[3];
} struct3_T;
#endif /* typedef_struct3_T */

#endif
/* End of code generation (esekf19_app_types.h) */
