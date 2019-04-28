#include <NewPing.h>

#define SONAR_NUM 3      // Number or sensors.
#define MAX_DISTANCE 200 // Max distance in cm.
#define PING_INTERVAL 80 //33 // Milliseconds between pings.

unsigned long pingTimer[SONAR_NUM]; // When each pings.
unsigned int cm[SONAR_NUM];         // Store ping distances.
uint8_t currentSensor = 0;          // Which sensor is active.

NewPing sonar[SONAR_NUM] = { // Sensor object array.
    NewPing(9, 10, MAX_DISTANCE),
    NewPing(7, 6, MAX_DISTANCE),
    NewPing(5, 4, MAX_DISTANCE)};

void setupSonar()
{
    pingTimer[0] = millis() + 75; // First ping start in ms.
    for (uint8_t i = 1; i < SONAR_NUM; i++)
        pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
}
void echoCheck()
{ // If ping echo, set distance to array.
    if (sonar[currentSensor].check_timer())
        cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

long lastSonarOutput;

void oneSensorCycle()
{ // Do something with the results.
    if (millis() - lastSonarOutput > 500)
    {
        for (uint8_t i = 0; i < SONAR_NUM; i++)
        {
            Serial3.print("$S_");
            Serial3.print(i);
            Serial3.print("=");
            Serial3.print(cm[i]);
            Serial3.println("");
            Serial.print("$S_");
            Serial.print(i);
            Serial.print("=");
            Serial.print(cm[i]);
            Serial.println("");
        }
        lastSonarOutput=millis();
    }
}

void loopSonar()
{
    for (uint8_t i = 0; i < SONAR_NUM; i++)
    {
        if (millis() >= pingTimer[i])
        {
            pingTimer[i] += PING_INTERVAL * SONAR_NUM;
            if (i == 0 && currentSensor == SONAR_NUM - 1)
                oneSensorCycle(); // Do something with results.
            sonar[currentSensor].timer_stop();
            currentSensor = i;
            cm[currentSensor] = 0;
            sonar[currentSensor].ping_timer(echoCheck);
        }
    }
    // The rest of your code would go here.
}
