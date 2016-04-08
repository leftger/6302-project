# 6.302 Midterm Project

## Segway for Mice

###Components:
. Teensy 3.1 Development Board
. SparkFun LSM9DS1 IMU Board
. Pololu DRV8835 Motor Control Board

This repository holds information pertaining to the development of a self-balancing robot.  The main components include a Teensy 3.2 board, a LSM9DS1 6-axis IMU, and a Pololu DRV8835 dual bidirectional motor driver.  The Teensy implements a PID controller by taking in the acceleration and gyroscopic measurements from the IMU and obtaining degree measurements.  The IMU is first calibrated by letting it stand still for about 2 seconds, after which the IMU should be able to provide accurate degree measurements.  Afterwards the Teensy uses PID control to maintain the robot balanced vertically.  As an extra 4 potentiometers are connected and sampled to obtain values for proportional, integral, and derivative gains and the desired angle which the robot should maintain.  This facilitates tuning the PID controller and provides intuitive feedback to the user as to the usefulness of feedback control. 