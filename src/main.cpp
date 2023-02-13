#include <Arduino.h>
#include <ESP32Servo.h>
#include <ICM_20948.h>

// Tuning parameters
#define COUPLING_SENSITIVITY 1.5
// -y direction on board is roll towards front
// +x direction on board is pitch towards left

// 16 servo objects can be created on the ESP32
Servo leftServo, rightServo;
// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33
int leftServoPin = 13;
int rightServoPin = 14;
int pos = 0;

// 9-Axis Gyroscope interface, REMEMBER TO UNCOMMENT LINE 29 OF ICM_20948_C.h
#define AD0_VAL 1
ICM_20948_I2C myICM;
double r, p, y;

void gyroSetup()
{
  Wire.begin();
  Wire.setClock(400000);


  bool initialized = false;
  while (!initialized)
  {
    // Initialize the ICM-20948
    myICM.begin(Wire, AD0_VAL);

    Serial.print(F("Initialization of the sensor returned: "));
    Serial.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      Serial.println(F("Trying again..."));
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }
  Serial.println(F("Device connected!"));


  bool success = true; // Use success to show if the DMP configuration was successful

  // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
  // Enable the DMP Game Rotation Vector sensor
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
  // Configuring DMP to output data at multiple ODRs:
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); // Set to the maximum


  // Enable the FIFO
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
  // Enable the DMP
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
  // Reset DMP
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
  // Reset FIFO
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  // Check success
  if (success)
  {
    Serial.println(F("DMP enabled!"));
  }
  else
  {
    Serial.println(F("Enable DMP failed!"));
    Serial.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
    while (1)
      ; // Do nothing more
  }
}

void servoSetup()
{
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  leftServo.setPeriodHertz(50);
  leftServo.attach(leftServoPin, 500, 2400);
  rightServo.setPeriodHertz(50);
  rightServo.attach(rightServoPin, 500, 2400);
}

bool gyroLoop(double* y, double* p, double* r)
{
  // Read any DMP data waiting in the FIFO
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);

  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
  {
    if ((data.header & DMP_header_bitmap_Quat6) > 0) // We have asked for GRV data so we should receive Quat6
    {
      // Scale to +/- 1
      double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30


      // Convert the quaternions to Euler angles (roll, pitch, yaw)
      // https://en.wikipedia.org/w/index.php?title=Conversion_between_quaternions_and_Euler_angles&section=8#Source_code_2

      double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

      double q2sqr = q2 * q2;

      // roll (x-axis rotation)
      double t0 = +2.0 * (q0 * q1 + q2 * q3);
      double t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
      *r = atan2(t0, t1) * 180.0 / PI;

      // pitch (y-axis rotation)
      double t2 = +2.0 * (q0 * q2 - q3 * q1);
      t2 = t2 > 1.0 ? 1.0 : t2;
      t2 = t2 < -1.0 ? -1.0 : t2;
      *p = asin(t2) * 180.0 / PI;

      // yaw (z-axis rotation)
      double t3 = +2.0 * (q0 * q3 + q1 * q2);
      double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
      *y = atan2(t3, t4) * 180.0 / PI;

      Serial.print(F("Roll:"));
      Serial.print(*r, 1);
      Serial.print(F(" Pitch:"));
      Serial.print(*p, 1);
      Serial.print(F(" Yaw:"));
      Serial.println(*y, 1);
    }
  }

  return true;
}

void setup()
{
  // Serial debugging interface
  Serial.begin(115200);
  while (!Serial)
    delay(10);
  Serial.println("Device starting up!");

  gyroSetup();
  servoSetup();
}

void loop()
{
  bool gyroSuccess = false;
  gyroSuccess = gyroLoop(&y, &p, &r);
  if (gyroSuccess)
  {
    // Roll maps to DOF1, Pitch maps to DOF2. For now, use 1 motor only.
    int pos_dof1 = COUPLING_SENSITIVITY*r + 90.0;
    if (pos_dof1 < 0)   pos_dof1 = 5;
    if (pos_dof1 > 180) pos_dof1 = 175;

    Serial.print("r: ");
    Serial.println(r);
    Serial.print("pos1: ");
    Serial.println(pos_dof1);

    leftServo.write(pos_dof1);
    rightServo.write(pos_dof1);

    if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
    {
      delay(10);
    }
  }
}