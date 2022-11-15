#ifndef __MPU_H__
#define __MPU_H__

/* ----- INCLUDES ----- */
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

/* ----- DEFINE ----- */
#define MPU6050_ACCEL_RANGE   MPU6050_RANGE_8_G       /* 2_G / 4_G / 8_G / 16_G */ 
#define MPU6050_GYRO_RANGE    MPU6050_RANGE_500_DEG   /* 250_DEG / 500_DEG / 1000_DEG / 2000_DEG */
#define MPU6050_BAND          MPU6050_BAND_21_HZ      /* 260_HZ / 184_HZ / 94_HZ / 44_HZ / 21_HZ / 10_HZ / 5_HZ */

/* ----- VARIABLES ----- */
sensors_event_t accel, gyro, temp;

#endif /* __MPU_H__ */