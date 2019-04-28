#include "SpotLed.h"

#include <Adafruit_NeoPixel.h>

Adafruit_NeoPixel pixels(7,30,NEO_GBR + NEO_KHZ800);

long startTime;

void setupLed() {
    pixels.begin();
    pixels.clear();
    for(int p=0;p<3;p++) {
    for(int o=180;o<360+180;o+=1) {
    for(int i=0;i<1;i++) {
        int v=cos(PI/180*o)*10+10;
        pixels.setPixelColor(i,pixels.Color(v,v,v));
    }
    pixels.show();
    delayMicroseconds(50);
    }
    delay(10);
    }
}

void loopLed() {
    if(startTime==0) startTime=millis();

    int v=sin(PI/180*(millis()-startTime)/10.0)*10+10;
    for(int i=0;i<1;i++) {
        pixels.setPixelColor(i,pixels.Color(0,v,0));
    }
    pixels.show();
}