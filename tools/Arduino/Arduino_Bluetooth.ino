/* This code retrives magnetometer data from MPU9250.
   This code is originated from Kris Winer's original MPU9250 library
   created on April 1, 2014.

  Date Modified  Modified by     Changes
  Feb 07, 2017   Vipul Vishnoi   Initial commit
  Feb 10, 2017   Vipul Vishnoi   Added support for multiple IMU's

  Hardware setup:
  MPU9250 Breakout --------- Arduino
  VDD ---------------------- 3.3V
  VDDI --------------------- 3.3V
  SDA ---------------------- A4
  SCL ---------------------- A5
  AD0 ---------------------- D8/D13
  GND ---------------------- GND

  HC-06--------------------- Arduino
  VCC ---------------------- 3.3V
  GND ---------------------- GND
  TXD ---------------------- RX
  RXD ---------------------- TX

  To enable serial port on bluetooth communication:
  ~$ sudo hcitool scan
  Scanning ...
  00:02:C7:7D:F5:17  HC-06
  ~$ sudo rfcomm bind /dev/rfcomm0 <Device MAC address> 1
  ~$ ls -l /dev/rfcomm0
  crw-rw---- 1 root dialout 216, 0 2008-12-14 23:15 /dev/rfcomm0
*/

#include "MPU9250.h"

// Pin definitions for IMU
int IMU[2] = {8, 13};
int IMU_count =  sizeof(IMU) / sizeof(int);
int IMU_ready[2];
int IMU_ready_count = 0;

MPU9250 myIMU;
char msg[60];

void setup()
{
  Wire.begin();
  // TWBR = 12;  // 400 kbit/sec I2C speed
  Serial.begin(9600);
  for ( int i = 0; i < IMU_count; i++)
  {
    pinMode(IMU[i], OUTPUT);
    digitalWrite(IMU[i], LOW);
  }
  // Disable BYPASS for all IMU
  myIMU.writeByte(0x69, INT_PIN_CFG, 0x20);
  myIMU.writeByte(0x68, INT_PIN_CFG, 0x20);
  // I2Cscan();

  // Initialize all IMU pins and perform self test
  for ( int i = 0; i < IMU_count; i++)
  {
    digitalWrite(IMU[i], HIGH);
    delay(50);

    // Read the WHO_AM_I register, this is a good test of communication
    byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
    Serial.print("MPU9250 "); Serial.print(i + 1); Serial.print(" - I AM "); Serial.println(c, HEX);

    if (c == 0x71) // WHO_AM_I should always be 0x68
    {
      sprintf(msg, "IMU at PIN %i is online", IMU[i]);
      Serial.println(msg);
      // Start by performing self test and reporting values
      myIMU.MPU9250SelfTest(myIMU.SelfTest);
      //      Serial.print("x-axis self test: acceleration trim within : ");
      //      Serial.print(myIMU.SelfTest[0], 1); Serial.println("% of factory value");
      //      Serial.print("y-axis self test: acceleration trim within : ");
      //      Serial.print(myIMU.SelfTest[1], 1); Serial.println("% of factory value");
      //      Serial.print("z-axis self test: acceleration trim within : ");
      //      Serial.print(myIMU.SelfTest[2], 1); Serial.println("% of factory value");
      //      Serial.print("x-axis self test: gyration trim within : ");
      //      Serial.print(myIMU.SelfTest[3], 1); Serial.println("% of factory value");
      //      Serial.print("y-axis self test: gyration trim within : ");
      //      Serial.print(myIMU.SelfTest[4], 1); Serial.println("% of factory value");
      //      Serial.print("z-axis self test: gyration trim within : ");
      //      Serial.print(myIMU.SelfTest[5], 1); Serial.println("% of factory value");

      // Calibrate gyro and accelerometers, load biases in bias registers
      myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

      myIMU.initMPU9250();

      // Read the WHO_AM_I register of the magnetometer, this is a good test of
      // communication
      byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
      Serial.print("AK8963 "); Serial.print("I AM "); Serial.println(d, HEX);

      // Get magnetometer calibration from AK8963 ROM
      myIMU.initAK8963(myIMU.magCalibration);
      // Initialize device for active mode read of magnetometer
      //
      //        Serial.println("Calibration values: ");
      //        Serial.print("X-Axis sensitivity adjustment value ");
      //        Serial.println(myIMU.magCalibration[0], 2);
      //        Serial.print("Y-Axis sensitivity adjustment value ");
      //        Serial.println(myIMU.magCalibration[1], 2);
      //        Serial.print("Z-Axis sensitivity adjustment value ");
      //        Serial.println(myIMU.magCalibration[2], 2);
      IMU_ready[IMU_ready_count] = IMU[i];
      IMU_ready_count += 1;
    } // if (c == 0x71)
    else
    {
      sprintf(msg, "No IMU at PIN %i", IMU[i]);
      Serial.println(msg);
      digitalWrite(IMU[i], LOW);
    }

    myIMU.getMres();
    // User environmental x-axis correction in milliGauss, should be
    // automatically calculated
    myIMU.magbias[0] = +470.;
    // User environmental x-axis correction in milliGauss TODO axis??
    myIMU.magbias[1] = +120.;
    // User environmental x-axis correction in milliGauss
    myIMU.magbias[2] = +125.;

    myIMU.writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x20);
    delay(40);
    digitalWrite(IMU[i], LOW);
    delay(10);
  }
}

void loop()
{
  for ( int i = 0; i < IMU_ready_count; i++)
  {
    digitalWrite(IMU_ready[i], HIGH);
    delay(10);
    myIMU.writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
    delay(40);

    // If intPin goes high, all data registers have new data
    // On interrupt, check if data ready interrupt
    if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
    {
      myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
      //

      // Calculate the magnetometer values in milliGauss
      // Include factory calibration per data sheet and user environmental
      // corrections
      // Get actual magnetometer value, this depends on scale being set
      myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes * myIMU.magCalibration[0] - myIMU.magbias[0];
      myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes * myIMU.magCalibration[1] - myIMU.magbias[1];
      myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes * myIMU.magCalibration[2] - myIMU.magbias[2];
    }

    // Must be called before updating quaternions!
    Serial.print(i + 1); Serial.print(" ");
    //    Serial.print((int)myIMU.mx); Serial.print(" ");
    //    Serial.print((int)myIMU.my); Serial.print(" ");
    //    Serial.print((int)myIMU.mz); Serial.println("");
    Serial.print((float)myIMU.mx, 3 ); Serial.print(" ");
    Serial.print((float)myIMU.my, 3 ); Serial.print(" ");
    Serial.print((float)myIMU.mz, 3 ); Serial.println(" ");
    myIMU.writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x20);
    delay(40);
    digitalWrite(IMU_ready[i], LOW);
    delay(10);
  }
}

void I2Cscan()
{
  // scan for i2c devices
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknow error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}
