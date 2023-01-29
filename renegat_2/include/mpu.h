#ifndef __MPU_H__
#define __MPU_H__

/* ================================================================
 * ===                        INCLUDES                          ===
 * ================================================================ */
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
// #include "MPU6050_6Axis_MotionApps20.h"

/* Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
 * is used in I2Cdev.h */
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

/* ================================================================
 * ===                         DEFINE                           ===
 * ================================================================ */
// #define MPU_USE_DMP

/* Offsets */
// #define MPU_CALIBRATION
#define X_ACCEL_OFFSET  -3924   /* Note : offsets are defined with the highest resolution (see Scale ranges) */
#define Y_ACCEL_OFFSET  -44
#define Z_ACCEL_OFFSET  813
#define X_GYRO_OFFSET   43
#define Y_GYRO_OFFSET   3
#define Z_GYRO_OFFSET   55

/* Scale ranges */
#define MPU6050_ACCEL_RANGE   MPU6050_ACCEL_FS_2    /* Accelerometer full-scale range : +/- 2, 4, 8 or 16g */
#define MPU6050_GYRO_RANGE    MPU6050_GYRO_FS_250   /* Accelerometer full-scale range : +/- 250, 500, 1000 or 2000 deg/s */

/* Computations */
// #define COMPUTE_EULER                 /* Takes approx. 0.7 ms to compute */
#define COMPUTE_YAWPITCHROLL          /* Takes approx. 0.9 ms to compute */
// #define COMPUTE_LOCAL_ACCELERATION    /* Real measured acceleration, adjusted to remove gravity. Takes approx. 0.25 ms to compute */
// #define COMPUTE_WORLD_ACCELERATION    /* Acceleration according to world frame (based on quaternions), adjusted to remove gravity. Takes approx. 0.45 ms to compute  */

/* Display */
// #define MPU_DISPLAY_QUATERNIONS
// #define MPU_DISPLAY_ACCEL
// #define MPU_DISPLAY_GYRO
// #define MPU_DISPLAY_EULER
#define MPU_DISPLAY_YAWPITCHROLL
// #define MPU_DISPLAY_ACCEL_LOCAL
// #define MPU_DISPLAY_ACCEL_WORLD



/* ================================================================
 * ===                        VARIABLES                         ===
 * ================================================================ */


/* ================================================================
 * ===                        FUNCTIONS                         ===
 * ================================================================ */
bool mpuSetup();
void mpuGetData();
void mpuDisplayData();

#endif /* __MPU_H__ */