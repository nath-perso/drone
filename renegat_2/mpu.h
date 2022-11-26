#ifndef __MPU_H__
#define __MPU_H__

/* ================================================================
 * ===                        INCLUDES                          ===
 * ================================================================ */
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"

/* Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
 * is used in I2Cdev.h */
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

/* ================================================================
 * ===                         DEFINE                           ===
 * ================================================================ */
/* Offsets */
#define X_ACCEL_OFFSET  -3924
#define Y_ACCEL_OFFSET  -44
#define Z_ACCEL_OFFSET  813
#define X_GYRO_OFFSET   43
#define Y_GYRO_OFFSET   3
#define Z_GYRO_OFFSET   55
/* Scale ranges */
#define MPU6050_ACCEL_RANGE   MPU6050_ACCEL_FS_4    /* Accelerometer full-scale range : +/- 2, 4, 8 or 16g */
#define MPU6050_GYRO_RANGE    MPU6050_GYRO_FS_500   /* Accelerometer full-scale range : +/- 250, 500, 1000 or 2000 deg/s */
/* Computations */
#define COMPUTE_EULER
#define COMPUTE_YAWPITCHROLL
#define COMPUTE_LOCAL_ACCELERATION    /* Display real measured acceleration, adjusted to remove gravity */
#define COMPUTE_WORLD_ACCELERATION    /* Display acceleration according to world frame (based on quaternions), adjusted to remove gravity */
/* Display */
// #define MPU_DISPLAY_QUATERNIONS
// #define MPU_DISPLAY_ACCEL
// #define MPU_DISPLAY_GYRO
// #define MPU_DISPLAY_EULER
// #define MPU_DISPLAY_YAWPITCHROLL
#define MPU_DISPLAY_ACCEL_LOCAL
// #define MPU_DISPLAY_ACCEL_WORLD



/* ================================================================
 * ===                        VARIABLES                         ===
 * ================================================================ */
/* Orientation/motion vars */
Quaternion quat;        /* [w, x, y, z]         quaternion container                          */
VectorInt16 accel;      /* [x, y, z]            accel sensor measurements                     */
VectorInt16 gyro;       /* [x, y, z]            gyro sensor measurements                      */
VectorInt16 accelLocal; /* [x, y, z]            gravity-free accel sensor measurements        */
VectorInt16 accelWorld; /* [x, y, z]            world-frame accel sensor measurements         */
VectorFloat gravity;    /* [x, y, z]            gravity vector                                */
float euler[3];         /* [psi, theta, phi]    Euler angle container                         */
float ypr[3];           /* [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector   */


/* ================================================================
 * ===                        FUNCTIONS                         ===
 * ================================================================ */
bool mpu_setup();
void mpuGetData();

#endif /* __MPU_H__ */