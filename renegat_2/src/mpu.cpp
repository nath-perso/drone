/* ================================================================
 * ===                        INCLUDES                          ===
 * ================================================================ */
#include "mpu.h"
#include "pins.h"

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Nano, this is digital I/O pin 2.
   ========================================================================= */

/* ================================================================
 * ===                        VARIABLES                         ===
 * ================================================================ */

/* Class default I2C address is 0x68 (AD0 low)
 * AD0 high = 0x69
 * To change I2C adress, pass it as parameter :
 * MPU6050 mpu(0x69)
 */
MPU6050 mpu;

/* MPU control/status vars */
bool dmpReady = false;              /* set true if DMP init was successful */
uint8_t mpuIntStatus;               /* holds actual interrupt status byte from MPU */
uint8_t devStatus;                  /* return status after each device operation (0 = success, !0 = error) */
uint16_t packetSize;                /* expected DMP packet size (default is 42 bytes) */
uint16_t fifoCount;                 /* count of all bytes currently in FIFO */
uint8_t fifoBuffer[64];             /* FIFO storage buffer */
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high

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

/**
 * @brief   Set acceleration and gyroscopic offsets
 */
void mpuSetOffsets()
{
  mpu.setXAccelOffset(X_ACCEL_OFFSET);
  mpu.setYAccelOffset(Y_ACCEL_OFFSET);
  mpu.setZAccelOffset(Z_ACCEL_OFFSET);
  mpu.setXGyroOffset(X_GYRO_OFFSET);
  mpu.setYGyroOffset(Y_GYRO_OFFSET);
  mpu.setZGyroOffset(Z_GYRO_OFFSET);
}

/**
 * @brief   Indicates that MPU interrupt pin has gone high
 */
void dmpDataReady()
{
  mpuInterrupt = true;
}

/**
 * @brief   Set up the MPU6050 chip
 * @returns   True if chip identified and initialized
 */
bool mpuSetup()
{
  Serial.println("Initializing MPU6050...");

  /* Join I2C bus (I2Cdev library doesn't do this automatically) */
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); /* 400kHz I2C clock. Comment this line if having compilation difficulties */
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  /* Initialize MPU */
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  /* Verify connection */
  if (!mpu.testConnection())
  { /* ERR MNGT - MPU connection failed */
    Serial.println("MPU6050 connection failed");
    return false;
  }
  Serial.println("MPU6050 connection successful");

  /* Set full scale ranges */
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_RANGE);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_RANGE);

#ifdef MPU_USE_DMP
  /* Initialize DMP */
  Serial.println("Initializing DMP...");
  devStatus = mpu.dmpInitialize();
#endif

  /* Set offsets */
#ifdef MPU_CALIBRATION
  Serial.println("Calibration requested for MPU offsets. Place the device still on a flat surface. Press any key when ready...");
  while (Serial.available() && Serial.read())
    ; /* empty buffer */
  while (!Serial.available())
    ; /* wait for data */
  while (Serial.available() && Serial.read())
    ; /* empty buffer again */
  Serial.println("Calibrating MPU offsets...");
  mpuCalibrate();
#else
  Serial.println("Setting MPU offsets...");
  mpuSetOffsets();
  Serial.print("MPU offsets set to : ");
  mpu.PrintActiveOffsets();
#endif

#ifdef MPU_USE_DMP
  /* Verify it worked */
  if (devStatus == 0)
  {
    /* Turn on the DMP */
    Serial.println("Enabling DMP...");
    mpu.setDMPEnabled(true);

    /* Enable Arduino interrupt detection */
    Serial.print("Enabling interrupt detection... ");
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    /* Set our DMP Ready flag so the main loop() function knows it's okay to use it */
    Serial.println("DMP ready ! Waiting for first interrupt...");
    dmpReady = true;

    /* Get expected DMP packet size for later comparison */
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    /* ERR MNGT - MPU DMP initialization failed
     * 1 = initial memory load failed
     * 2 = DMP configuration updates failed
     * (if it's going to break, usually the code will be 1) */
    Serial.print("DMP Initialization failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
    return false;
  }
#endif

  Serial.println("MPU6050 ready to go");
  return true;
}

/**
 * @brief   Calibrate and set offsets
 */
void mpuCalibrate()
{
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  Serial.print("\t");
  mpu.PrintActiveOffsets();
}

/**
 * @brief   Get MPU accelerometer range value
 * @returns Accelerometer range (g)
 */
float mpuGetAccelRange()
{
  switch (mpu.getFullScaleAccelRange())
  {
  case 0x00:
    return 2;
    break;
  case 0x01:
    return 4;
    break;
  case 0x02:
    return 8;
    break;
  case 0x03:
    return 16;
    break;

  default:
    return 2;
    break;
  }
}

/**
 * @brief   Get MPU gyrometer range value
 * @returns Gyrometer range (g)
 */
float mpuGetGyroRange()
{
  switch (mpu.getFullScaleGyroRange())
  {
  case 0x00:
    return 250;
    break;
  case 0x01:
    return 500;
    break;
  case 0x02:
    return 1000;
    break;
  case 0x03:
    return 2000;
    break;

  default:
    return 250;
    break;
  }
}

/**
 * @brief   Compute Roll Pitch and Yaw using complementary filter
 * @returns [yaw, pitch, roll]
 */
void mpuComputeRollPitchYawComplementaryFilter(VectorInt16 accel_data, VectorInt16 gyro_data, float *out_data)
{
  static unsigned long old_time = 0;
  static float pitch_cf = 0;
  static float roll_cf = 0;
  static float yaw = 0;

  static float gyro_roll_stored = 0;
  static float gyro_pitch_stored = 0;

  float dt = (micros() - old_time) * 1e-6;
  old_time = micros();

  float cutoff_frequency = 0.5; /* Complementary filter cutoff frequency (Hz). The lower the cutoff, the more we trust the gyro. */
  float K = (1 / (2 * PI * cutoff_frequency)) / (1 / (2 * PI * cutoff_frequency) + dt);

  /* Convert acceleration data to floats */
  float f_accel_data_x = accel_data.x;
  float f_accel_data_y = accel_data.y;
  float f_accel_data_z = accel_data.z;

  /* Compute accelerometer estimations for the complementary filter */
  float accel_roll = atan2(f_accel_data_y, f_accel_data_z);
  float accel_pitch = atan2(-f_accel_data_x, sqrt(f_accel_data_y * f_accel_data_y + f_accel_data_z * f_accel_data_z));

  /* Compute conversion factor */
  int gyro_range = mpuGetGyroRange();
  float cnts_to_radps = DEG2RAD * gyro_range / MPU6050_HALF_RANGE_COUNTS;

  /* Compute gyro estimations for the complementary filter  */
  float gyro_roll_cf = roll_cf + gyro_data.x * dt * cnts_to_radps;
  float gyro_pitch_cf = pitch_cf + gyro_data.y * dt * cnts_to_radps;

  /* Compute gyro standalone estimations (for comparison) */
  float gyro_roll = gyro_roll_stored + gyro_data.x * dt * cnts_to_radps;
  float gyro_pitch = gyro_pitch_stored + gyro_data.y * dt * cnts_to_radps;

  gyro_roll_stored = gyro_roll;
  gyro_pitch_stored = gyro_pitch;

  pitch_cf = K * gyro_pitch_cf + (1 - K) * accel_pitch;
  roll_cf = K * gyro_roll_cf + (1 - K) * accel_roll;

  /* Compute yaw angle (gyro based -> it will drift) */
  float gyro_yaw = yaw + gyro_data.z * dt * cnts_to_radps;
  yaw = gyro_yaw;

  /* DEBUG DISPLAY */
  // Serial.println(K, 4);              Serial.print('\t');
  // Serial.print(accel_roll * RAD2DEG);Serial.print('\t');
  // Serial.print(gyro_roll * RAD2DEG); Serial.print('\t');
  // Serial.print(roll_cf * RAD2DEG);   Serial.print('\t');
  // Serial.print(accel_pitch*RAD2DEG); Serial.print('\t');
  // Serial.print(gyro_pitch*RAD2DEG);  Serial.print('\t');
  // Serial.print(pitch_cf*RAD2DEG);    Serial.print('\t');
  // Serial.print(yaw*RAD2DEG);         Serial.print('\t');
  // Serial.print('\n');

  /* Ouptut data */
  out_data[0] = yaw;
  out_data[1] = pitch_cf;
  out_data[2] = roll_cf;
}

/**
 * @brief   Get MPU data
 * @returns 0 if successful
 */
void mpuGetData()
{
#ifdef MPU_USE_DMP
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
  { /* Get the latest packet from DMP. Takes approx. 3 ms */

    /* Get DMP data */
    mpu.dmpGetQuaternion(&quat, fifoBuffer); /* Note : getting quaternions is required to compute other data */
    mpu.dmpGetAccel(&accel, fifoBuffer);
    mpu.dmpGetGyro(&gyro, fifoBuffer);

    /* Compute additional data */
#ifdef COMPUTE_EULER
    mpu.dmpGetEuler(euler, &quat);
#endif
#ifdef COMPUTE_YAWPITCHROLL
    mpu.dmpGetGravity(&gravity, &quat);
    mpu.dmpGetYawPitchRoll(ypr, &quat, &gravity);
#endif
#if defined(COMPUTE_LOCAL_ACCELERATION) || defined(COMPUTE_WORLD_ACCELERATION)
    mpu.dmpGetGravity(&gravity, &quat);
    mpu.dmpGetLinearAccel(&accelLocal, &accel, &gravity);
#ifdef COMPUTE_WORLD_ACCELERATION
    mpu.dmpGetLinearAccelInWorld(&accelWorld, &accelLocal, &quat);
#endif
#endif
  }
  else
  {
    Serial.println("Failed to get current FIFO Packet from MPU DMP");
  }
#else
  mpu.getMotion6(&accel.x, &accel.y, &accel.z, &gyro.x, &gyro.y, &gyro.z);
#ifdef COMPUTE_YAWPITCHROLL
  mpuComputeRollPitchYawComplementaryFilter(accel, gyro, ypr);
#endif
#endif
}

/**
 * @brief   Display MPU data depending on defined configuration
 */
void mpuDisplayData()
{
#ifdef MPU_DISPLAY_QUATERNIONS
  Serial.print("Quaternions\t");
  Serial.print(quat.w);
  Serial.print("\t");
  Serial.print(quat.x);
  Serial.print("\t");
  Serial.print(quat.y);
  Serial.print("\t");
  Serial.println(quat.z);
#endif
#ifdef MPU_DISPLAY_ACCEL
  Serial.print("Raw_acceleration\t");
  Serial.print(accel.x);
  Serial.print("\t");
  Serial.print(accel.y);
  Serial.print("\t");
  Serial.println(accel.z);
#endif
#ifdef MPU_DISPLAY_GYRO
  Serial.print("Raw_gyro\t");
  Serial.print(gyro.x);
  Serial.print("\t");
  Serial.print(gyro.y);
  Serial.print("\t");
  Serial.println(gyro.z);
#endif
#ifdef MPU_DISPLAY_EULER
  Serial.print("Euler_angles\t");
  Serial.print(euler[0] * 180 / M_PI);
  Serial.print("\t");
  Serial.print(euler[1] * 180 / M_PI);
  Serial.print("\t");
  Serial.println(euler[2] * 180 / M_PI);
#endif
#ifdef MPU_DISPLAY_YAWPITCHROLL
  Serial.print("Yaw/Pitch/Roll\t");
  Serial.print(ypr[0] * 180 / M_PI);
  Serial.print("\t");
  Serial.print(ypr[1] * 180 / M_PI);
  Serial.print("\t");
  Serial.println(ypr[2] * 180 / M_PI);
#endif
#ifdef MPU_DISPLAY_ACCEL_LOCAL
  Serial.print("Local_acceleration\t");
  Serial.print(accelLocal.x);
  Serial.print("\t");
  Serial.print(accelLocal.y);
  Serial.print("\t");
  Serial.println(accelLocal.z);
#endif
#ifdef MPU_DISPLAY_ACCEL_WORLD
  Serial.print("World_acceleration\t");
  Serial.print(accelWorld.x);
  Serial.print("\t");
  Serial.print(accelWorld.y);
  Serial.print("\t");
  Serial.println(accelWorld.z);
#endif
}