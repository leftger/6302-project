#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include <SparkFunLSM9DS1.h>

LSM9DS1 imu;

#define LSM9DS1_M  0x1E // 
#define LSM9DS1_AG  0x6B //


char message[100];
float Kp = 0;
float Ki = 0;
float Kd = 0;
float offset = 0;

float Kp_scaler = 5;
float Ki_scaler = 1;
float Kd_scaler = 5;
float offset_scaler = 0.01;

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
    //acc_pitch = -atan2(ay,az)*180/PI;
    acc_roll = -atan2(ax,az)*180/PI + 90;
    //predicted_pitch = alpha*(predicted_pitch + gx*dt)+(1-alpha)*acc_pitch;
    predicted_roll = alpha*(predicted_roll - gy*dt) + (1-alpha)*acc_roll;
    //angle_into +=predicted_roll*dt;

    Serial.print("gx: ");
    Serial.print(gx);
    Serial.print(" gy: ");
    Serial.print(gy);
    Serial.print(" gz: ");
    Serial.println(gz);
  }
  float pitch(){
    return predicted_pitch;
  }
  float angle(){
    return predicted_roll;
  }


};


Angle angle;


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
}

void loop() {
  angle.update();
}



