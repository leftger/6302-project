/***************************************************************************
 * 6.033 Spring 2016 Midterm Project
 * Grant Gunnison and Gerzain Mata
 * Segway for Mice -- adapted from Adafruit 9DOF code and
 * MIT's Joe Steinmeyer's sample IMU code
 * Creation Date: April 2nd 2013
 * 
 * This Arduino sketch drives a Texas Instruments dual directional motor
 * board and a Adafruit 9DOF IMU and gets accelerometer readings from
 * the IMU.  This should then give the Teensy board the ability to
 * compensate for any imbalance and drive the motor in a corresponding
 * direction.
 * 
 * TEENSY    ------------ Adafruit 9DOF
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
 * Adafruit 9DOF Breakout Version: 1.0
 **************************************************************************/
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_Simple_AHRS.h>
#include <math.h>

// Create sensor instances.
Adafruit_LSM303_Accel_Unified accel(30301);
Adafruit_LSM303_Mag_Unified mag(30302);
Adafruit_L3GD20_Unified gyro(20);

// Create simple AHRS algorithm using the above sensors.
Adafruit_Simple_AHRS          ahrs(&accel, &mag);

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
#define ALPHA 20

int ledPin = 13;      // LED connected to digital pin 9
int phasePin1 = 22;
int phasePin2 = 20;
int dutyPin1 = 23;
int dutyPin2 = 21;
int val = 0;        // variable to store the read value

boolean dir = true; // direction

void setup()
{
  Serial.begin(115200);
  Serial.println(F("Adafruit 9 DOF Board AHRS Example")); Serial.println("");
  
  // Initialize the sensors.
  accel.begin();
  mag.begin();
  gyro.enableAutoRange(true);
  gyro.begin();

  // setup for the motor driver pins
  pinMode(phasePin1, OUTPUT);   // sets the pin as output
  pinMode(phasePin2, OUTPUT);
  pinMode(dutyPin1, OUTPUT);
  pinMode(dutyPin2, OUTPUT);
}

void loop(void)
{
//  if(val >= 255){
//    mode = false;
//    dir = !dir;
//  }
//  else if(val <=190){
//    mode = true;
//    dir = !dir;
//  }
  
  sensors_vec_t   orientation;
  sensors_event_t event;
  gyro.getEvent(&event);

  // Use the simple AHRS function to get the current orientation.
  if (ahrs.getOrientation(&orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields */
    Serial.print(F("Orientation: "));
    Serial.print(orientation.roll);
    Serial.print(F(" "));

    val = 100 + orientation.roll;

    Serial.print(F("Val: "));
    Serial.println(abs(val));
    
    analogWrite(dutyPin1, ALPHA * abs(val));  // analogRead values go from 0 to 1023, analogWrite values from 0 to 255
    digitalWrite(phasePin1, val >0 ? HIGH : LOW);
    Serial.print(F("Dir1: "));
    Serial.print(digitalRead(phasePin1));
    analogWrite(dutyPin2, ALPHA * abs(val));
    digitalWrite(phasePin2, val >0 ? HIGH : LOW);
    Serial.print(F(" Dir2: "));
    Serial.println(digitalRead(phasePin2));
    
//    Serial.print(orientation.pitch);
//    Serial.print(F(" "));
//    Serial.print(orientation.heading);
//    Serial.println(F(""));
  }

   /* Display the results (speed is measured in rad/s) */
  Serial.print("X: "); Serial.print(event.gyro.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.gyro.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.gyro.z); Serial.print("  ");
  Serial.println("rad/s ");
  
  delay(5);
//  if(mode){
//    val++;
//  }
//  else val--;
}
