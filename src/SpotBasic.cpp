#if defined(ARDUINO_AVR_MEGA2560)

#include "SpotBasic.h"

Point::Point() {}

Point::Point(float x, float y, float z)
{
  this->x = x;
  this->y = y;
  this->z = z;
}

float Point::GetDistance(Point point1, Point point2)
{
  return sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2) + pow(point1.z - point2.z, 2));
}

void OLED::Init()
{
}

void OLED::Update()
{
}

void SonarSensor::Init(int echo_pin, int trigger_pin)
{
}

void PowerLed::Init()
{
}

void PowerLed::Update() {}

void IMU::Init() {}
void IMU::Update() {}
void FrontLight::Init()
{
  this->pixel = new Adafruit_NeoPixel(7, this->pin, NEO_GBR + NEO_KHZ800);
  this->pixel->begin();
  this->pixel->clear();
  for (int p = 0; p < 3; p++)
  {
    for (int o = 180; o < 360 + 180; o += 1)
    {
      for (int i = 0; i < 1; i++)
      {
        int v = cos(PI / 180 * o) * 10 + 10;
        this->pixel->setPixelColor(i, this->pixel->Color(v, v, v));
      }
      this->pixel->show();
      delayMicroseconds(50);
    }
    delay(10);
  }
}

void FrontLight::Update()
{
  if (this->startTime == 0)
    this->startTime = millis();

  int v = sin(PI / 180 * (millis() - this->startTime) / 10.0) * 10 + 10;
  for (int i = 0; i < 1; i++)
  {
    this->pixel->setPixelColor(i, this->pixel->Color(0, v, 0));
  }
  this->pixel->show();
}

SpotMicro::SpotMicro() {}

void SpotMicro::Start()
{
  pinMode(A2, INPUT);
  Serial.begin(9600);
  Serial2.begin(9600);
  Serial3.begin(9600);
  delay(3000);
  Serial.println("MicroSpot v.01");
  Serial.print("max. Memory-Size: ");
  Serial.println(memory.getMaxSize());
  delay(300);
  // Init Bluetooth
  Serial3.println("AT+");
  delay(500);
  Serial.println("Getting Bluetooth version");
  Serial3.println("AT+VERSION");
  delay(200);
  while (Serial3.available() > 0)
  {
    Serial.write(Serial3.read());
  }
  Serial.println("initializing Bluetooth Console");
  Serial3.println("AT+BAUD6");
  //  Serial3.begin(57600);

  delay(500);

  while (Serial3.available() > 0)
  {
    Serial.write(Serial3.read());
  }
  Serial3.println("AT+NAMEMICROSPOT");
  while (Serial3.available() > 0)
  {
    Serial.write(Serial3.read());
  }
  Serial.println("Initializing OLED");
  oled.Init();
  Serial.println("Initializing SONAR");
  sonarFrontLeft.Init(PinConfig::SONAR_1_ECHO_PIN, PinConfig::SONAR_1_TRIGGER_PIN);
  sonarFrontRight.Init(PinConfig::SONAR_2_ECHO_PIN, PinConfig::SONAR_2_TRIGGER_PIN);
  sonarBottomFront.Init(PinConfig::SONAR_3_ECHO_PIN, PinConfig::SONAR_3_TRIGGER_PIN);
  sonarBottomRear.Init(PinConfig::SONAR_4_ECHO_PIN, PinConfig::SONAR_4_TRIGGER_PIN);
  Serial.println("Initializing PowerLED");

  powerLed.Init();
  Serial.println("Initializing IMU");
  imu.Init();
  Serial.println("Initializing FrontLED");

  light.Init();
  Serial.println("Initialization completed");
}

void PowerManager::switchMode(MODE newmode)
{
  if (this->mode != newmode)
  {
    this->mode = newmode;
    Serial.print("Modeswitch:");
    Serial.println(newmode);
  }
}

void PowerManager::Init()
{
  this->samples = 0;
  this->sum = 0;
  this->samples_timer = 0;
}

void PowerManager::Update()
{
  if (millis() - this->samples_timer > 100)
  {
    int a = analogRead(A2);
    this->samples++;
    this->sum += a;
    if (this->samples > 10)
    {
      this->samples = 0;
      this->sum = 0;
    }
    samples_timer = millis();
  }
  float voltage = ((float)this->sum / (float)this->samples * 5.01) / 1024.0;
  if(this->samples<10||voltage<5.0) {
    switchMode(MODE_BRAIN_ONLY);
  } else {
    switchMode(MODE_FULL);
  }
}

float PowerManager::getVoltage()
{
  float voltage = ((float)this->sum / (float)this->samples * 5.01) / 1024.0;
  return voltage * 5.45454545;
}

void SpotMicro::Update()
{
  this->powerManager.Update();
  //  oled.Update();
  powerLed.Update();
  imu.Update();
  light.Update();
  ticks++;

  // Debug output for ticks per second and voltage
  if (millis() - timer1 > 1000)
  {
    long m = millis();
    Serial.print("TPS:");
    Serial.println(ticks);
    timer1 = millis();
    ticks = 0;
    Serial.print("V:");
    Serial.println(this->powerManager.getVoltage());
  }
}

#endif