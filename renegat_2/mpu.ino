#include "mpu.h"

Adafruit_MPU6050 mpu;

/* !
 * @brief   Set up the MPU6050 chip
 * @returns   True if chip identified and initialized
 */
bool mpu_setup() {
  Serial.println("MPU6050 setup");

  if (!mpu.begin()) {   /* ERR MNGT - MPU not found */
    Serial.println("Failed to find MPU6050 chip");
    return false;
  }

  Serial.println("MPU6050 found");
   
  mpu.setAccelerometerRange(MPU6050_ACCEL_RANGE);
  mpu.setGyroRange(MPU6050_GYRO_RANGE);
  mpu.setFilterBandwidth(MPU6050_BAND);

  Serial.println("MPU6050 ready to go");
  return true;
}

/* !
 * @brief   Get acceleration, gyroscopic and temperature data 
 */
void mpu_get_data() {
  mpu.getEvent(&accel, &gyro, &temp);
}


/* !
 * @brief   Display acceleration, gyroscopic and temperature data 
 */
void mpu_display_data() {
  Serial.print(temp.temperature);
  Serial.print(",");
  Serial.print(accel.acceleration.x);
  Serial.print(","); Serial.print(accel.acceleration.y);
  Serial.print(","); Serial.print(accel.acceleration.z);
  Serial.print(",");
  Serial.print(gyro.gyro.x);
  Serial.print(","); Serial.print(gyro.gyro.y);
  Serial.print(","); Serial.print(gyro.gyro.z);
  Serial.println();
}


