/***************************************************************************
 * 6.033 Spring 2016 Midterm Project
 * Grant Gunnison and Gerzain Mata
 * Segway for Mice -- adapted from SparkFun LSM9DS1 code and
 * MIT's Joe Steinmeyer's sample IMU code
 * Creation Date: April 2nd 2016
 * 
 * This Arduino sketch drives a Texas Instruments dual directional motor
 * board and a SparkFun LSM9DS1 IMU and gets accelerometer readings from
 * the IMU.  This should then give the Teensy board the ability to
 * compensate for any imbalance and drive the motor in a corresponding
 * direction.
 * 
 * TEENSY    ------------ Pololu DRV8835
 * A9 (PWM)  ------------ ENB (PWM Motor 2)
 * A8        ------------ PHB (Direction Motor 2)
 * A7        ------------ ENA (PWM Motor 1)
 * A6        ------------ PHA (Direction Motor 1)
 * 
 * TEENSY    ------------ SparkFun LSM9DS1
 * A5 (SCL0) ------------ SCL
 * A4 (SDA0) ------------ SDA0
 * 
 * TEENSY    ------------ Potentiometers
 * A3        ------------ Kp Gain
 * A2        ------------ Kd Gain
 * A1        ------------ Ki Gain
 * A0        ------------ Desired
 * 
 * Development Environment Specifics:
 * IDE: Arduino 1.6.7 with Teensyduino package installed with Homebrew
 * Hardware Platform: Teensy 3.1
 * SparkFun LSM9DS1 Breakout Version: 1.0
 **************************************************************************/
#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include <SparkFunLSM9DS1.h>

LSM9DS1 imu;

#define LSM9DS1_M  0x1E // 
#define LSM9DS1_AG  0x6B //

#define ledPin 13      // LED connected to digital pin 9
#define phasePin1 22
#define phasePin2 20
#define dutyPin1 23
#define dutyPin2 21
#define Kp_pin A3
#define Kd_pin A2
#define Ki_pin A1
#define desired_pin A0
#define outMax 255
#define outMin -255
#define SLIDING_WINDOW_SIZE 5

float desired = 0;

float theta = 0;
float error;
float measurement[2];
float error_integral = 0;
float error_derivative = 0;

char message[100];
float Kp = 0;
float Ki = 0;
float Kd = 0;
float offset = 0;

float Kp_scaler = 180;
float Ki_scaler = 1;
float Kd_scaler = 5;
float desired_scaler = 180;

class SlidingWindowAvg
{
  float bins[SLIDING_WINDOW_SIZE];
  int counter;
  float total;
  public:
  SlidingWindowAvg(){
    this->counter = 0;
    this->total = 0;
  }
  void add(float num){
    total -= bins[counter];
    bins[counter] = num;
    total+= bins[counter];
    counter++;
    if(counter >= SLIDING_WINDOW_SIZE){
      counter = 0;
    }
  }
  float getAvg(){
    return total / SLIDING_WINDOW_SIZE;
  }
};

class KalmanFilter
{
  float q; // process noise covariance
  float r; // measurement noise covariance
  float p; // estimation error covariance
  float x; // value
  float k; // kalman gain

  public:
  KalmanFilter(float q, float r, float p, float x){
    this->q = q; // .125
    this->r = r; // 3
    this->p = p; // .75
    this->x = x; // 0
  }

  void update(float measurement){
    // prediction update
    p = p + q;

    // measurement update
    k = p / (p + r);
    x = x + k * (measurement - x);
    p = (1 - k) * p;
  }

  float getMeasurement(){
    return x;
  }
};

class Angle
{
  float ax_cal;
  float ay_cal;
  float az_cal;
  float gx_cal;
  float gy_cal;
  float gz_cal;

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
    ax_cal=0;
    ay_cal=0;
    az_cal=0;
    gx_cal=0;
    gy_cal=0;
    gz_cal=0;
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

  void calibrate(){

    Serial.println("Don't touch");
    Serial.println("");
    Serial.print("Calibrating...");
    for (int i = 0; i<100; i++){
      imu.readAccel();
      imu.readGyro();
      ax_cal+=0.01*imu.ax;
      ay_cal+=0.01*imu.ay;
      az_cal+=0.01*imu.az;
  
      gx_cal+=0.01*imu.gx;
      gy_cal+=0.01*imu.gy;
      gz_cal+=0.01*imu.gz;
      delay(30);
      Serial.print('.'); 
    }
    ax_cal-=16384.0; //gravitycorrection assuming 2g full scale!!!  change if change scale

    Serial.println(ax_cal);
    Serial.println(ay_cal);
    Serial.println(az_cal);
    Serial.println(gx_cal);
    Serial.println(gy_cal);
    Serial.println(gz_cal);
    delay(1800); 
  }
  void update(){
    new_time = millis();
    dt = (new_time-last_time)*0.001; //in seconds
    last_time = new_time;
    imu.readAccel();
    imu.readGyro();
    float ax=imu.ax-ax_cal;
    float ay=imu.ay-ay_cal;
    float az=imu.az-az_cal;
    //Gyro:
    float gx=(imu.gx-gx_cal)*gyro_scale;//into dps
    float gy=(imu.gy-gy_cal)*gyro_scale;
    float gz=(imu.gz-gz_cal)*gyro_scale;
    acc_pitch = -atan2(ay,az)*180/PI;
    acc_roll = -atan2(ax,az)*180/PI + 90;
    predicted_pitch = alpha*(predicted_pitch + gx*dt)+(1-alpha)*acc_pitch;
    predicted_roll = alpha*(predicted_roll - gy*dt) + (1-alpha)*acc_roll;
    //angle_into +=predicted_roll*dt;
//      Serial.print("Pitch: ");
//      Serial.print(pitch());
//      Serial.print(" Roll: ");
//      Serial.println(angle());
  }
  float pitch(){
    return predicted_pitch;
  }
  float angle(){
    return predicted_roll;
  }
  float deltat(){
    return dt;
  }
};

Angle angle;
//KalmanFilter kfilter(.125, 3 ,.75, 0);
SlidingWindowAvg desiredBin;
SlidingWindowAvg kpBin;
SlidingWindowAvg kdBin;
SlidingWindowAvg kiBin;

void setup() {
  Serial.begin(115200);
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  Serial.println("Initializing the LSM9DS1");
  if (!imu.begin())
  {
    while (1){
      Serial.println("Comm Failure with LSM9DS1");
      delay(500);
    }
  }
  Serial.println("Setup Good");
  angle.calibrate();

  // setup for the motor driver pins
  pinMode(phasePin1, OUTPUT);   // sets the pin as output
  pinMode(phasePin2, OUTPUT);
  pinMode(dutyPin1, OUTPUT);
  pinMode(dutyPin2, OUTPUT);
  pinMode(Kd_pin, INPUT);
  pinMode(Ki_pin, INPUT);
  pinMode(Kp_pin, INPUT);
  pinMode(desired_pin, INPUT);
  analogReference(DEFAULT);
}

void loop() {
  angle.update();
  //kfilter.update(angle.pitch());
  measurement[0] = measurement[1];
  //measurement[1] = kfilter.getMeasurement();
  measurement[1] = angle.pitch();
  getPIDGainsAndDesired();
  //error = desired - kfilter.getMeasurement(); // error with kalman filter
  error = desired - angle.pitch(); // current error
  error_integral += error * angle.deltat() * Ki;
  if(error_integral > outMax) error_integral = outMax;
  else if(error_integral < outMin) error_integral = outMin;
  error_derivative = (measurement[1] - measurement[0]) / angle.deltat(); 
  theta = Kp*error + error_integral - Kd*error_derivative;
  if(theta > outMax) theta = outMax;
  else if(theta < outMin) theta = outMin;
  // Print actual output angle to Arduino Serial Plotter
  serialSendSystemData();
  analogWrite(dutyPin1, abs(theta));  // analogRead values go from 0 to 1023, analogWrite values from 0 to 255
  digitalWrite(phasePin1, theta >0 ? HIGH : LOW);
  analogWrite(dutyPin2, abs(theta));
  digitalWrite(phasePin2, theta >0 ? HIGH : LOW);
}

void getPIDGainsAndDesired(){
  desiredBin.add(float(analogRead(desired_pin)) / 1023);
  desired = desired_scaler * desiredBin.getAvg();
  kpBin.add(float(analogRead(Kp_pin)) / 1023);
  Kp = Kp_scaler * kpBin.getAvg();
  kdBin.add(float(analogRead(Kd_pin)) / 1023);
  Kd = Kd_scaler * kdBin.getAvg();
  kiBin.add(float(analogRead(Ki_pin)) / 1023);
  Ki = Ki_scaler * kiBin.getAvg();
}

void serialSendSystemData(){
  Serial.print("Measured theta: ");
  Serial.print(measurement[1]);
  Serial.print(" Output Motor value: ");
  Serial.print(theta);
  Serial.print(" Desired: ");
  Serial.print(desired);
  Serial.print(" Kp: ");
  Serial.print(Kp);
  Serial.print(" Kd: ");
  Serial.print(Kd);
  Serial.print(" Ki: ");
  Serial.println(Ki);
}

