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
#define RAD2DEG     180/PI
#define DEG2RAD     PI/180


/* ================================================================
 * ===                        VARIABLES                         ===
 * ================================================================ */


/* ================================================================
 * ===                        FUNCTIONS                         ===
 * ================================================================ */
bool mpuSetup();
void mpuGetData();
void mpuDisplayData();
float mpuGetRoll();
float mpuGetPitch();
float mpuGetYaw();
float mpuGetYawSpeed();

#endif /* __MPU_H__ */