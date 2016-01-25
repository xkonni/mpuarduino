#include <Wire.h>
#include <math.h>

#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C
// REGISTERS
#define    REG_MAG_CNTL1              0x0A
#define    REG_INT_PIN_CFG            0x37
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
char printBuf[256];


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
 * get accelerometer and gyroscope sensor values
 */
void getAccelGyro(vector<int16_t> *a, vector<int16_t> *g) {
  // Read accelerometer and gyroscope
  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);

  // Create 16 bits values from 8 bits data
  // .. Accelerometer
  a->x=-(Buf[0]<<8 | Buf[1]);
  a->y=-(Buf[2]<<8 | Buf[3]);
  a->z=Buf[4]<<8 | Buf[5];
  // .. Gyroscope
  g->x=-(Buf[8]<<8 | Buf[9]);
  g->y=-(Buf[10]<<8 | Buf[11]);
  g->z=Buf[12]<<8 | Buf[13];
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
  m->x=-(Mag[3]<<8 | Mag[2]);
  m->y=-(Mag[1]<<8 | Mag[0]);
  m->z=-(Mag[5]<<8 | Mag[4]);
}

/*
 * get the current, tilt-compensated, heading
 */
double getTiltHeading(vector<int16_t> *a, vector<int16_t> *m) {
  double heading;

  // convert to [-1; 1]
  vector<double> *realA = new vector<double>();
  realA->x = (double) a->x / (1 << 15) * ACC_SCALE;
  realA->y = (double) a->y / (1 << 15) * ACC_SCALE;
  realA->z = (double) a->z / (1 << 15) * ACC_SCALE;

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

  float pitch = asin(-realA->x);
  float roll = asin(realA->y/cos(pitch));
  float xh = m->x * cos(pitch) + m->z * sin(pitch);
  float yh = m->x * sin(roll) * sin(pitch) + m->y * cos(roll) - m->z * sin(roll) * cos(pitch);
  // unused
  // float zh = -m->x * cos(roll) * sin(pitch) + m->y * sin(roll) + m->z * cos(roll) * cos(pitch);

  heading = round(180 * atan2(yh, xh)/PI);
  if (heading < 0) heading += 360;

  return (int(heading));
}

/*
 * get the current heading, accurate when the sensor is lying flat on a surface
 */
int16_t getHeading(vector<int16_t> *m) {
  int16_t heading;

  double x = m->x;
  double y = m->y;
  heading = 180 * atan2(y, x) / PI;
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
  I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_1000_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_4_G);
  // DEBUG
  ACC_SCALE=1;
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
  delay(1000);
}

/*
 * arduino loop function
 */
void loop() {
  // collect samples accel and gyro
  getAccelGyro(accel, gyro);
  filter(accel, accel_flt, 0.5);
  filter(gyro, gyro_flt, 0.5);
  // collect samples from mag
  getMag(mag);

  /*
   * Display time & all values
   */
  // sprintf(printBuf, "[%8ld] accel: %6d/%6d/%6d gyro: %6d/%6d/%6d mag:%6d/%6d/%6d th: %3d, h: %3d",
  //     millis() - ti, accel->x, accel->y, accel->z,
  //     gyro->x, gyro->y, gyro->z, mag->x, mag->y, mag->z,
  //     int(getTiltHeading(accel, mag)), getHeading(mag));

  /*
   * Display time & accel, mag, tilt heading, heading
   */
  sprintf(printBuf, "[%8ld] [accel %6d/%6d/%6d] [mag%6d/%6d/%6d] [th %3d, h %3d]",
      millis() - ti, accel->x, accel->y, accel->z, mag->x, mag->y, mag->z,
      int(getTiltHeading(accel, mag)), getHeading(mag));

  /*
   * Display time & tilt heading & heading
   */
  // sprintf(printBuf, "[%8ld] th: %3d, h: %3d",
  //     millis() - ti, int(getTiltHeading(accel, mag)), getHeading(mag));

  /*
   * Display time & heading
   */
  // sprintf(printBuf, "[%8ld] h: %3d", millis() - ti, getHeading(mag));

  /*
   * Display time & tilt heading
   */
  // sprintf(printBuf, "[%8ld] h: %3d",
  //     millis() - ti, int(getTiltHeading(accel, mag)));

  // print
  Serial.println(printBuf);
  // wait a bit before restarting the loop
  delay(100);
}
