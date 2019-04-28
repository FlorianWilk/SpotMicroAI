#include "SpotButton.h"
#include <Arduino.h>
#define POWERBUTTON_PIN 45

void SpotButton::init()
{
    pinMode(POWERBUTTON_PIN, OUTPUT);
    dimtimer = millis();
}

void SpotButton::update()
{
      long diff=millis()-dimtimer;
        analogWrite(POWERBUTTON_PIN, cos(PI / 180.0 * (diff/10.0)) * 127 + 128);
}