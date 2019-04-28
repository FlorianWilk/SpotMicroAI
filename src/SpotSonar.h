/*
  SpotSonar.h

  SonarWrapper for MicroSpot

*/
#ifndef SpotSonar_h
#define SpotSonar_h
#include <NewPing.h>
#include "Arduino.h"
#define SONAR_NUM 3      // Number or sensors.
#define MAX_DISTANCE 200 // Max distance in cm.
#define PING_INTERVAL 33 // Milliseconds between pings.

#define SONAR_NUM 3 // Number or sensors.

void setupSonar();
void loopSonar();

#endif