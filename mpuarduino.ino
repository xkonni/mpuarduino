#include <Wire.h>
#include <math.h>
#include <SpeedTrig.h>

#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C
// REGISTERS
#define    REG_MAG_CNTL1              0x0A
#define    REG_INT_PIN_CFG            0x37
#define    REG_ACCEL_XOUT_H           0x3B
#define    REG_ACCEL_XOUT_L           0x3C
#define    REG_ACCEL_YOUT_H           0x3D
#define    REG_ACCEL_YOUT_L           0x3E
#define    REG_ACCEL_ZOUT_H           0x3F
#define    REG_ACCEL_ZOUT_L           0x40
#define    REG_TEMP_OUT_H             0x41
#define    REG_TEMP_OUT_L             0x42
#define    REG_GYRO_XOUT_H            0x43
#define    REG_GYRO_XOUT_L            0x44
#define    REG_GYRO_YOUT_H            0x45
#define    REG_GYRO_YOUT_L            0x46
#define    REG_GYRO_ZOUT_H            0x47
#define    REG_GYRO_ZOUT_L            0x48
#define    REG_MAG_XOUT_L             0x03
#define    REG_MAG_XOUT_H             0x04
#define    REG_MAG_YOUT_L             0x05
#define    REG_MAG_YOUT_H             0x06
#define    REG_MAG_ZOUT_L             0x07
#define    REG_MAG_ZOUT_H             0x08
// MASKS, VALUES
#define    MASK_BYPASS_ENABLE         0x02

#define    GYRO_FULL_SCALE_250_DPS    0x00
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

/*
 * global variables
 */
// scale used for the accelerometer: 2 to 8
int ACC_SCALE;
// Initial time
long int ti;
// Output
char printBuf[128];


/*
 * template vector
 */
template <typename T> struct vector {
  T x;
  T y;
  T z;
};

/*
 * read Nbytes from the register "Register" on I2C device at address "Address"
 * update the *Data pointer with the read values
 */
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data) {
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();

  // Read Nbytes
  Wire.requestFrom(Address, Nbytes);
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}

/*
 * read single from the register "Register" on I2C device at address "Address"
 * return the data
 */
uint8_t I2CreadByte(uint8_t Address, uint8_t Register) {
  uint8_t data;
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();

  // Read byte
  Wire.requestFrom(Address, (uint8_t) 1);
  if (Wire.available())
    data=Wire.read();

  return(data);
}

/*
 * write a byte "Data" in device on the I2C device at "Address" at "Register"
 */
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data) {
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

/*
 * write a single bit
 */
void I2CwriteBit(uint8_t Address, uint8_t Register, uint8_t mask, boolean enabled) {
  // get current register value
  uint8_t data;
  I2Cread(Address, Register, 1, &data);
  // set bit
  if (enabled) data = data | mask;
  else data = data & ~mask;
  // write
  I2CwriteByte(Address, Register, data);
}

/*
 * get accelerometer sensor values
 */
void getAccel (vector<int16_t> *a) {
  // read accelerometer
  a->x = -(I2CreadByte(MPU9250_ADDRESS, REG_ACCEL_XOUT_H) << 8 |
      I2CreadByte(MPU9250_ADDRESS, REG_ACCEL_XOUT_L));
  a->y = -(I2CreadByte(MPU9250_ADDRESS, REG_ACCEL_YOUT_H) << 8 |
      I2CreadByte(MPU9250_ADDRESS, REG_ACCEL_YOUT_L));
  a->z = -(I2CreadByte(MPU9250_ADDRESS, REG_ACCEL_ZOUT_H) << 8 |
      I2CreadByte(MPU9250_ADDRESS, REG_ACCEL_ZOUT_L));
}

/*
 * get gyroscope sensor values
 */
void getGyro (vector<int16_t> *g) {
  // read gyroscope
  // - ?
  g->x = (I2CreadByte(MPU9250_ADDRESS, REG_GYRO_XOUT_H) << 8 |
      I2CreadByte(MPU9250_ADDRESS, REG_GYRO_XOUT_L));
  // - ?
  g->y = (I2CreadByte(MPU9250_ADDRESS, REG_GYRO_YOUT_H) << 8 |
      I2CreadByte(MPU9250_ADDRESS, REG_GYRO_YOUT_L));
  g->z = (I2CreadByte(MPU9250_ADDRESS, REG_GYRO_ZOUT_H) << 8 |
      I2CreadByte(MPU9250_ADDRESS, REG_GYRO_ZOUT_L));
}

/*
 * get magnetometer values
 */
void getMag(vector<int16_t> *m) {
  // enable bypass
  I2CwriteBit(MAG_ADDRESS, REG_INT_PIN_CFG, MASK_BYPASS_ENABLE, true);
  // enable single measurement
  I2CwriteByte(MAG_ADDRESS, REG_MAG_CNTL1, 0x01);

  // wait for measurement to complete
  uint8_t ST1;
  do {
    I2Cread(MAG_ADDRESS,0x02,1,&ST1);
  } while (!(ST1&0x01));

  // read magnetometer data
  uint8_t Mag[7];
  I2Cread(MAG_ADDRESS,0x03,7,Mag);

  // create 16 bits values from 8 bits data
  m->x=(Mag[1]<<8 | Mag[0]);
  m->y=(Mag[3]<<8 | Mag[2]);
  m->z=(Mag[5]<<8 | Mag[4]);
}

/*
 * get the current, tilt-compensated, heading
 * !!! WARNING !!!
 * ! while it runs fine at the beginning, something breaks at some point
 * ! in execution. use with caution!
 * !!! WARNING !!!
 */
int16_t getTiltHeading(vector<int16_t> *a, vector<int16_t> *m) {
  int16_t heading;

  // convert to [-1; 1]
  vector<float> *realA = new vector<float>();
  realA->x = (float) a->x / (1 << 15) * ACC_SCALE;
  realA->y = (float) a->y / (1 << 15) * ACC_SCALE;
  realA->z = (float) a->z / (1 << 15) * ACC_SCALE;

  // DEBUG warnings
  // if ((realA->x < -1) || (realA->x > 1)) {
  //   Serial.println(realA->x, 5);
  //   Serial.print("realA->x fail ");
  //   // realA->x = realA->x/2;
  // }
  // if ((realA->y < -1) || (realA->y > 1)) {
  //   Serial.println(realA->y, 5);
  //   Serial.print("realA->y fail ");
  //   // realA->y = realA->y/2;
  // }
  // if ((realA->z < -1) || (realA->z > 1)) {
  //   Serial.println(realA->z, 5);
  //   Serial.print("realA->z fail ");
  //   realA->z = realA->z/2;
  // }

  // float pitch = asin(-realA->x);
  float pitch = -SpeedTrig.acos(-realA->x) + PI/2;
  float sin_pitch = SpeedTrig.sin(pitch);
  float cos_pitch = SpeedTrig.cos(pitch);
  // float roll = asin(realA->y/cos_pitch);
  float roll = -SpeedTrig.acos(realA->y/cos_pitch) + PI/2;
  float cos_roll = SpeedTrig.cos(roll);
  float sin_roll = SpeedTrig.sin(roll);

  float xh = m->x * cos_pitch + m->z * sin_pitch;
  float yh = m->x * sin_roll * sin_pitch + m->y * cos_roll - m->z * sin_roll * cos_pitch;

  // float pitch = asin(-realA->x);
  // float roll = asin(realA->y/cos(pitch));
  // float xh = m->x * cos(pitch) + m->z * sin(pitch);
  // float yh = m->x * sin(roll) * sin(pitch) + m->y * cos(roll) - m->z * sin(roll) * cos(pitch);
  // unused
  // float zh = -m->x * cos(roll) * sin(pitch) + m->y * sin(roll) + m->z * cos(roll) * cos(pitch);
  // heading = round(180 * atan2(yh, xh)/PI);

  heading = int(round(180 * SpeedTrig.atan2(yh, xh)/PI));
  if (heading < 0) heading += 360;

  return (heading);
}

/*
 * get the current heading, accurate when the sensor is lying flat on a surface
 */
int16_t getHeading(vector<int16_t> *m) {
  int16_t heading;

  double h_tmp;

  double x, y;
  if ((m->x == 0) && (m->y == 0)) {
    Serial.println("avoid 0 0");
    x = 1;
  }
  else {
    x = m->x;
    y = m->y;
  }
  h_tmp = atan2(y,x);
  heading = (h_tmp * 180) / PI;
  if (heading < 0) heading += 360;

  return (heading);
}

/*
 * filter the current sample to smoothe out spikes in the readings
 */
void filter(vector<int16_t> *sample, vector<int16_t> *filtered, double coeff) {
  // apply filter
  sample->x = (sample->x * (1 - coeff)) + (filtered->x * coeff);
  sample->y = (sample->y * (1 - coeff)) + (filtered->y * coeff);
  sample->z = (sample->z * (1 - coeff)) + (filtered->z * coeff);
  // also overwrite last filtered values
  filtered->x = sample->x;
  filtered->y = sample->y;
  filtered->z = sample->z;
}

/*
 * initialize the MPU9250
 */
void MPU9250_init() {
  // Set accelerometers low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS,29,0x06);
  // Set gyroscope low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS,26,0x06);
  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_250_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_2_G);
  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
  // Request continuous magnetometer measurements in 16 bits
  I2CwriteByte(MAG_ADDRESS,0x0A,0x16);
}

vector<int16_t> *accel_flt = new vector<int16_t>();
vector<int16_t> *gyro_flt  = new vector<int16_t>();
vector<int16_t> *mag_flt   = new vector<int16_t>();
vector<int16_t> *accel = new vector<int16_t>();
vector<int16_t> *gyro  = new vector<int16_t>();
vector<int16_t> *mag   = new vector<int16_t>();

/*
 * arduino setup function
 */
void setup()
{
  // Arduino initializations
  Wire.begin();
  Serial.begin(38400);
  MPU9250_init();
  // index = 0;
  ti = millis();
  delay(100);
}

/*
 * arduino loop function
 */
void loop() {
  // collect accel samples
  getAccel(accel);
  // run filter
  filter(accel, accel_flt, 0.5);

  // collect gyro samples
  getGyro(gyro);
  // run filter
  filter(gyro, gyro_flt, 0.5);

  // collect samples from mag
  getMag(mag);
  // run filter
  filter(mag, mag_flt, 0.5);

  /*
   * Display time & all values
   */
  // sprintf(printBuf, "[%10ld] accel: %6d/%6d/%6d gyro: %6d/%6d/%6d mag:%6d/%6d/%6d th: %3d, h: %3d",
  //     millis() - ti, accel->x, accel->y, accel->z,
  //     gyro->x, gyro->y, gyro->z, mag->x, mag->y, mag->z,
  //     int(getTiltHeading(accel, mag)), getHeading(mag));

  /*
   * Display
   */
  /* just print accel as x,y,z,heading for plotplane.py */
  // sprintf(printBuf, "%06d,%06d,%06d,%03d", accel->x, accel->y, accel->z, getHeading(mag));
  /* just print accel as x,y,z, mag as x,y,z */
  sprintf(printBuf, " %06d,%06d,%06d,%06d,%06d,%06d",
      accel->x, accel->x, accel->z, mag->x, mag->y, mag->z);

  /* print multiple elements */
  // clear
  // sprintf(printBuf, "");
  // add time
  // sprintf(printBuf, "%s [%10ld]", printBuf, millis() - ti);
  // add accel: x/y/z
  // sprintf(printBuf, "%s accel: %6d/%6d/%6d", printBuf, accel->x, accel->y, accel->z);
  // add gyro: x/y/z
  // sprintf(printBuf, "%s gyro: %6d/%6d/%6d", printBuf, gyro->x, gyro->y, gyro->z);
  // add mag: x/y/z
  // sprintf(printBuf, "%s mag: %6d/%6d/%6d", printBuf, mag->x, mag->y, mag->z);
  // add heading h: deg
  // sprintf(printBuf, "%s h: %d", printBuf, getHeading(mag));
  // add tilt compensated heading th: deg
  // sprintf(printBuf, "%s th: %d", printBuf, getTiltHeading(accel, mag));

  // print
  Serial.println(printBuf);
  // wait a bit before restarting the loop
  delay(100);
}
