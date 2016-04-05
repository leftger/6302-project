/***************************************************************************
 * 6.033 Spring 2016 Midterm Project
 * Grant Gunnison and Gerzain Mata
 * Segway for Mice -- adapte from Jim Linblom's SparkFun LSM9DS1's code and
 * MIT's Joe Steinmeyer's sample IMU code
 * Creation Date: April 2nd 2013
 * 
 * This Arduino sketch drives a Texas Instruments dual directional motor
 * board and a SparkFun LSM9DS1 IMU and gets accelerometer readings from
 * the IMU.  This should then give the Teensy board the ability to
 * compensate for any imbalance and drive the motor in a corresponding
 * direction.
 * 
 * TEENSY    ------------ LSM9DS1
 * A9 (PWM)  ------------ ENB (PWM Motor 2)
 * A8        ------------ PHB (Direction Motor 2)
 * A7        ------------ ENA (PWM Motor 1)
 * A6        ------------ PHA (Direction Motor 1)
 * A5 (SCL0) ------------ SCL
 * A4 (SDA0) ------------ SDA0
 * 
 * Development Environment Specifics:
 * IDE: Arduino 1.6.7 with Teensyduino package installed with Homebrew
 * Hardware Platform: Teensy 3.1
 * LSM9DS1 Breakout Version: 1.0
 **************************************************************************/
#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include <SparkFunLSM9DS1.h>

LSM9DS1 imu;

#define LSM9DS1_M  0x1E // 
#define LSM9DS1_AG  0x6B //

/* Possibly useful 6.302 variables -- no use right now
* char message[100];
* float Kp = 0;
* float Ki = 0;
* float Kd = 0;
* float offset = 0;
*
* float Kp_scaler = 5;
* float Ki_scaler = 1;
* float Kd_scaler = 5;
* float offset_scaler = 0.01;
*/

// Global variables to keep track of update rates
unsigned long startTime;
unsigned int accelReadCounter = 0;
unsigned int gyroReadCounter = 0;
unsigned int magReadCounter = 0;

// Global variables to print to serial monitor at a steady rate
unsigned long lastPrint = 0;
const unsigned int PRINT_RATE = 500;

int ledPin = 13;      // LED connected to digital pin 9
int phasePin1 = 22;
int phasePin2 = 20;
int dutyPin1 = 23;
int dutyPin2 = 21;
int val = 127;        // variable to store the read value

boolean mode = true; // count up
boolean dir = true;

class Angle
{
  //float accel_scale = 32768.0/
  float gyro_scale = 245.0/32768.0;
  
  float acc_pitch;
  float gyro_pitch;
  float acc_roll;
  float gyro_roll;
  float predicted_pitch;
  float predicted_roll;
  unsigned long last_time;
  unsigned long new_time;
  float dt;
  float alpha = 0.99;

  float angle_into;
  float derv;
  float erroro;

  public:
  Angle(){
    acc_pitch=0;
    gyro_pitch=0;
    acc_roll=0;
    gyro_roll=0;
    last_time = 0;
    new_time = 0;
    dt = 0;
    predicted_pitch=0;
    predicted_roll=0;

    angle_into = 0;
    derv = 0;
    erroro = 0;
    
  }
    
  void update(){
    new_time = millis();
    dt = (new_time-last_time)*0.001; //in seconds
    last_time = new_time;
    imu.readAccel();
    imu.readGyro();
    float ax=imu.calcAccel(imu.ax);
    float ay=imu.calcAccel(imu.ay);
    float az=imu.calcAccel(imu.az);
    //Gyro:
    float gx=imu.calcGyro(imu.gx);//into dps
    float gy=imu.calcGyro(imu.gy);
    float gz=imu.calcGyro(imu.gz);
    //acc_pitch = -atan2(ay,az)*180/PI;
    acc_roll = -atan2(ax,az)*180/PI + 90;
    //predicted_pitch = alpha*(predicted_pitch + gx*dt)+(1-alpha)*acc_pitch;
    predicted_roll = alpha*(predicted_roll - gy*dt) + (1-alpha)*acc_roll;
    angle_into +=predicted_roll*dt;
  }
  
  float pitch(){
    return predicted_pitch;
  }
  float angle(){
    return predicted_roll;
  }
};


Angle angle;


uint16_t initLSM9DS1()
{
  setupDevice(); // Setup general device parameters
  setupGyro(); // Set up gyroscope parameters
  setupAccel(); // Set up accelerometer parameters
  setupMag(); // Set up magnetometer parameters
  
  return imu.begin();
}

void setupDevice()
{
  // we're using I2C for the IMU
  imu.settings.device.commInterface = IMU_MODE_I2C;
  // setting the I2C address
  imu.settings.device.mAddress = LSM9DS1_M; // Use I2C addres 0x1E
  // [agAddress] sets the I2C address or SPI CS pin of the
  // LSM9DS1's accelerometer/gyroscope.
  imu.settings.device.agAddress = LSM9DS1_AG; // I2C address 0x6B
}

void setupGyro()
{
  // [enabled] turns the gyro on or off.
  imu.settings.gyro.enabled = true;  // Enable the gyro
  // [scale] sets the full-scale range of the gyroscope.
  // scale can be set to either 245, 500, or 2000
  imu.settings.gyro.scale = 245; // Set scale to +/-245dps
  // [sampleRate] sets the output data rate (ODR) of the gyro
  // sampleRate can be set between 1-6
  // 1 = 14.9    4 = 238
  // 2 = 59.5    5 = 476
  // 3 = 119     6 = 952
  imu.settings.gyro.sampleRate = 3; // 59.5Hz ODR
  // [bandwidth] can set the cutoff frequency of the gyro.
  // Allowed values: 0-3. Actual value of cutoff frequency
  // depends on the sample rate. (Datasheet section 7.12)
  imu.settings.gyro.bandwidth = 0;
  // [lowPowerEnable] turns low-power mode on or off.
  imu.settings.gyro.lowPowerEnable = false; // LP mode off
  // [HPFEnable] enables or disables the high-pass filter
  imu.settings.gyro.HPFEnable = true; // HPF disabled
  // [HPFCutoff] sets the HPF cutoff frequency (if enabled)
  // Allowable values are 0-9. Value depends on ODR.
  // (Datasheet section 7.14)
  imu.settings.gyro.HPFCutoff = 1; // HPF cutoff = 4Hz
  // [flipX], [flipY], and [flipZ] are booleans that can
  // automatically switch the positive/negative orientation
  // of the three gyro axes.
  imu.settings.gyro.flipX = false; // Don't flip X
  imu.settings.gyro.flipY = false; // Don't flip Y
  imu.settings.gyro.flipZ = false; // Don't flip Z
}

void setupAccel()
{
  // [enabled] turns the acclerometer on or off.
  imu.settings.accel.enabled = true; // Enable accelerometer
  // [enableX], [enableY], and [enableZ] can turn on or off
  // select axes of the acclerometer.
  imu.settings.accel.enableX = true; // Enable X
  imu.settings.accel.enableY = true; // Enable Y
  imu.settings.accel.enableZ = true; // Enable Z
  // [scale] sets the full-scale range of the accelerometer.
  // accel scale can be 2, 4, 8, or 16
  imu.settings.accel.scale = 8; // Set accel scale to +/-8g.
  // [sampleRate] sets the output data rate (ODR) of the
  // accelerometer. ONLY APPLICABLE WHEN THE GYROSCOPE IS
  // DISABLED! Otherwise accel sample rate = gyro sample rate.
  // accel sample rate can be 1-6
  // 1 = 10 Hz    4 = 238 Hz
  // 2 = 50 Hz    5 = 476 Hz
  // 3 = 119 Hz   6 = 952 Hz
  imu.settings.accel.sampleRate = 1; // Set accel to 10Hz.
  // [bandwidth] sets the anti-aliasing filter bandwidth.
  // Accel cutoff freqeuncy can be any value between -1 - 3. 
  // -1 = bandwidth determined by sample rate
  // 0 = 408 Hz   2 = 105 Hz
  // 1 = 211 Hz   3 = 50 Hz
  imu.settings.accel.bandwidth = 0; // BW = 408Hz
  // [highResEnable] enables or disables high resolution 
  // mode for the acclerometer.
  imu.settings.accel.highResEnable = false; // Disable HR
  // [highResBandwidth] sets the LP cutoff frequency of
  // the accelerometer if it's in high-res mode.
  // can be any value between 0-3
  // LP cutoff is set to a factor of sample rate
  // 0 = ODR/50    2 = ODR/9
  // 1 = ODR/100   3 = ODR/400
  imu.settings.accel.highResBandwidth = 0;  
}

void setupMag()
{
  // [enabled] turns the magnetometer on or off.
  imu.settings.mag.enabled = true; // Enable magnetometer
  // [scale] sets the full-scale range of the magnetometer
  // mag scale can be 4, 8, 12, or 16
  imu.settings.mag.scale = 12; // Set mag scale to +/-12 Gs
  // [sampleRate] sets the output data rate (ODR) of the
  // magnetometer.
  // mag data rate can be 0-7:
  // 0 = 0.625 Hz  4 = 10 Hz
  // 1 = 1.25 Hz   5 = 20 Hz
  // 2 = 2.5 Hz    6 = 40 Hz
  // 3 = 5 Hz      7 = 80 Hz
  imu.settings.mag.sampleRate = 7; // Set OD rate to 80Hz
  // [tempCompensationEnable] enables or disables 
  // temperature compensation of the magnetometer.
  imu.settings.mag.tempCompensationEnable = false;
  // [XYPerformance] sets the x and y-axis performance of the
  // magnetometer to either:
  // 0 = Low power mode      2 = high performance
  // 1 = medium performance  3 = ultra-high performance
  imu.settings.mag.XYPerformance = 3; // Ultra-high perform.
  // [ZPerformance] does the same thing, but only for the z
  imu.settings.mag.ZPerformance = 3; // Ultra-high perform.
  // [lowPowerEnable] enables or disables low power mode in
  // the magnetometer.
  imu.settings.mag.lowPowerEnable = false;
  // [operatingMode] sets the operating mode of the
  // magnetometer. operatingMode can be 0-2:
  // 0 = continuous conversion
  // 1 = single-conversion
  // 2 = power down
  imu.settings.mag.operatingMode = 0; // Continuous mode
}


void setup()
{
  Serial.begin(115200);
  
  Serial.println("Initializing the LSM9DS1");
  uint16_t status = initLSM9DS1();
  Serial.print("LSM9DS1 WHO_AM_I's returned: 0x");
  Serial.println(status, HEX);
  Serial.println("Should be 0x683D");
  Serial.println();
  
  startTime = millis();


  // setup for the motor driver pins
  pinMode(phasePin1, OUTPUT);   // sets the pin as output
  pinMode(phasePin2, OUTPUT);
  pinMode(dutyPin1, OUTPUT);
  pinMode(dutyPin2, OUTPUT);
}

void loop()
{
  if(val >= 255){
    mode = false;
    dir = !dir;
  }
  else if(val <=190){
    mode = true;
    dir = !dir;
  }
  analogWrite(dutyPin1, val);  // analogRead values go from 0 to 1023, analogWrite values from 0 to 255
  digitalWrite(phasePin1, dir);
  analogWrite(dutyPin2, val);
  digitalWrite(phasePin2, !dir);

  doIMUStuff();
  
  
  
  delay(5);
  if(mode){
    val++;
  }
  else val--;
}

void doIMUStuff()
{
    // imu.accelAvailable() returns 1 if new accelerometer
  // data is ready to be read. 0 otherwise.
  if (imu.accelAvailable())
  {
    imu.readAccel();
    accelReadCounter++;
  }
  
  // imu.gyroAvailable() returns 1 if new gyroscope
  // data is ready to be read. 0 otherwise.
  if (imu.gyroAvailable())
  {
    imu.readGyro();
    gyroReadCounter++;
  }
  
  // imu.magAvailable() returns 1 if new magnetometer
  // data is ready to be read. 0 otherwise.
  if (imu.magAvailable())
  {
    imu.readMag();
    magReadCounter++;
  }

  // Every PRINT_RATE milliseconds, print sensor data:
  if ((lastPrint + PRINT_RATE) < millis())
  {
    //printSensorReadings();
    float runTime = (float)(millis() - startTime) / 1000.0;
    float gyroRate = (float)gyroReadCounter / runTime;
    float accelRate = (float)accelReadCounter / runTime;
    Serial.print("A: ");
    Serial.print(imu.calcAccel(imu.ax));
    Serial.print(", ");
    Serial.print(imu.calcAccel(imu.ay));
    Serial.print(", ");
    Serial.print(imu.calcAccel(imu.az));
    Serial.print(" g \t| ");
    Serial.print(accelRate);
    Serial.println(" Hz");
    Serial.print("G: ");
    Serial.print(imu.calcGyro(imu.gx));
    Serial.print(", ");
    Serial.print(imu.calcGyro(imu.gy));
    Serial.print(", ");
    Serial.print(imu.calcGyro(imu.gz));
    Serial.print(" dps \t| ");
    Serial.print(gyroRate);
    Serial.println(" Hz");
    lastPrint = millis();
  }
}



// printSensorReadings prints the latest IMU readings
// along with a calculated update rate.
void printSensorReadings()
{
  float runTime = (float)(millis() - startTime) / 1000.0;
  float accelRate = (float)accelReadCounter / runTime;
  float gyroRate = (float)gyroReadCounter / runTime;
  float magRate = (float)magReadCounter / runTime;
  Serial.print("A: ");
  Serial.print(imu.calcAccel(imu.ax));
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.ay));
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.az));
  Serial.print(" g \t| ");
  Serial.print(accelRate);
  Serial.println(" Hz");
  Serial.print("G: ");
  Serial.print(imu.calcGyro(imu.gx));
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gy));
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gz));
  Serial.print(" dps \t| ");
  Serial.print(gyroRate);
  Serial.println(" Hz");
  Serial.print("M: ");
  Serial.print(imu.calcMag(imu.mx));
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.my));
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.mz));
  Serial.print(" Gs \t| ");
  Serial.print(magRate);
  Serial.println(" Hz");
  Serial.println();
}
