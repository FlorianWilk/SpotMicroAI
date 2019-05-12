/*
File        Basic Classes for SpotMicro
Author      Florian Wilk / 2019
Copyright   Florian Wilk
License     Creative Commons Attribution ShareAlike 3.0
            (http://creativecommons.org/licenses/by-sa/3.0/legalcode)
*/

#pragma once
#if defined(ARDUINO_AVR_MEGA2560)

#include <Arduino.h>
#include <Servo.h>
#include <EEPROM.h>
//#include <FlexiTimer2.h>
#include <Adafruit_NeoPixel.h>
#include "SpotMemory.h"

#define LOG_TO_SERIAL1  True


class PinConfig {

    public:
        static const int NEOMATRIX_PIN=30;
        static const int POWERLED_PIN=45;
        static const int MPU_INTERRUPT_PIN=1;

        static const int SONAR_1_ECHO_PIN=4;
        static const int SONAR_1_TRIGGER_PIN=5;
        static const int SONAR_2_ECHO_PIN=6;
        static const int SONAR_2_TRIGGER_PIN=7;
        static const int SONAR_3_ECHO_PIN=8;
        static const int SONAR_3_TRIGGER_PIN=9;
        static const int SONAR_4_ECHO_PIN=10;
        static const int SONAR_4_TRIGGER_PIN=11;
};

class EepromAddresses
{
public:
  static const float controllerVersion = 0;

  static const float servo22 = 10;
  static const float servo23 = 12;
  static const float servo24 = 14;
  static const float servo25 = 16;
  static const float servo26 = 18;
  static const float servo27 = 20;
  static const float servo39 = 22;
  static const float servo38 = 24;
  static const float servo37 = 26;
  static const float servo36 = 28;
  static const float servo35 = 30;
  static const float servo34 = 32;

  static const float robotState = 50;
};

class Logger {
    public:
       static void e(String s);
       static void d(String s);
       static void w(String s);
       static void i(String s);
};

class Point
{
public:
  Point();
  Point(float x, float y, float z);

  static float GetDistance(Point point1, Point point2);

  volatile float x, y, z;
};




class SpotJoint
{
public:
  SpotJoint();
  void Set(int servoPin, float jointZero, bool jointDir, float jointMinAngle, float jointMaxAngle, int offsetAddress);

  void SetOffset(float offset);
  void SetOffsetEnableState(bool state);

  void RotateToDirectly(float jointAngle);

  float GetJointAngle(float servoAngle);

  bool CheckJointAngle(float jointAngle);

  volatile float jointAngleNow;
  volatile float servoAngleNow;

  static int firstRotateDelay;

private:
  Servo servo;
  int servoPin;
  float jointZero;
  bool jointDir;
  float jointMinAngle;
  float jointMaxAngle;
  int offsetAddress;
  volatile float offset = 0;
  volatile bool isOffsetEnable = true;
  volatile bool isFirstRotate = true;
};

class SpotLeg {
  private:
    SpotJoint shoulder;
    SpotJoint leg;
    SpotJoint foot;

  public:
    void moveLeg(int shoulder, int leg, int foot);
    void moveShoulder(int shoulder);
    void moveLeg(int leg);
    void moveFoot(int foot);
    
};

class PowerLed {
public:
    static const int pin = PinConfig::POWERLED_PIN;

    void Init();
    void Blink(int times,int speed,int intensity);
    void Update();
};

class IMU {
public:
    static const int pin = PinConfig::MPU_INTERRUPT_PIN;

    void Init();
    void Update();
};

class OLED {
public:
    void Init();
    void Update();
};

class SonarSensor {
private:
    int echoPin;
    int triggerPin;    
public:
    void Init(int echoPin,int triggerPin);
};

class FrontLight {
private:
    Adafruit_NeoPixel *pixel; 
    long startTime;
    int pin=PinConfig::NEOMATRIX_PIN;
public:
    void Init();
    void Update();
};

class SpotMicro {
private:
    int ticks=0;
    long timer1 = millis();
    Memory memory;


public:
    SpotMicro();
    void Start();
    void Update();

    SonarSensor sonarFrontLeft,sonarFrontRight,sonarBottomFront,sonarBottomRear;
    IMU imu;
    PowerLed powerLed;
    OLED oled;
    FrontLight light;
    SpotLeg leg_front_left,leg_front_right,leg_rear_left,leg_rear_right;

  enum State { Install, Calibrate, Boot, Action };
  State state = State::Boot;
};

#endif