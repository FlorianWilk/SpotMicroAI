/**

**/


#include <Servo.h>
#include <NewPing.h>
#include "SpotSonar.h"
#include "SpotDisplay.h"

#include "SpotButton.h"
#include "SpotMPU.h"
#include "SpotLed.h"
#include "SpotBasic.h"

#define PIN_BUTTON_4 38
#define PIN_BUTTON_3 39
#define PIN_BUTTON_2 40
#define PIN_BUTTON_1 41

#define PIN_VOLTAGE A0

Servo myservo;
SpotButton button;
SpotMicro spot;

void initButtons()
{
  button.init();
  pinMode(PIN_BUTTON_1, INPUT);
  pinMode(PIN_BUTTON_2, INPUT);
  pinMode(PIN_BUTTON_3, INPUT);
  pinMode(PIN_BUTTON_4, INPUT);
}

void setup()
{

  spot.Start();

  for (double i = 180; i < 2 * 360 + 270; i++)
  {
    analogWrite(45, cos(PI / 180.0 * i) * 127 + 128);
    delayMicroseconds(1000);
  }
 //setupLed();
//   setupDisplay();
//  splashScreen();


  setupSonar();
//  setupIMU();

  initButtons();

  //  myservo.attach(22);
  //myservo.detach();
}

void readButton(int button)
{
  long val = digitalRead(button);
  if (val > 0)
  {
    Serial3.println("BUTTON_1");
  }
}


void loop()
{
  spot.Update();
//  button.update();
//  loopDisplay();
  loopIMU();
 // loopSonar();
 loopLed();
  //readButton(PIN_BUTTON_1);
  //readButton(PIN_BUTTON_2);
  //readButton(PIN_BUTTON_3);
  //readButton(PIN_BUTTON_4);
 
}
