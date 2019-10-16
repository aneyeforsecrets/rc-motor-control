#include <Arduino.h>
// ask_transmitter.pde
// -*- mode: C++ -*-
// Simple example of how to use RadioHead to transmit messages
// with a simple ASK transmitter in a very simple way.
// Implements a simplex (one-way) transmitter with an TX-C1 module
// Tested on Arduino Mega, Duemilanova, Uno, Due, Teensy, ESP-12

#include <RH_ASK.h>
#ifdef RH_HAVE_HARDWARE_SPI
#include <SPI.h> // Not actually used but needed to compile
#endif

RH_ASK driver;
// RH_ASK driver(2000, 4, 5, 0); // ESP8266 or ESP32: do not use pin 11 or 2
// RH_ASK driver(2000, 3, 4, 0); // ATTiny, RX on D3 (pin 2 on attiny85) TX on D4 (pin 3 on attiny85),

// Joystick
const uint8_t kXPin = 7;
const uint8_t kYPin = 6;

// On-Board LED used as status indication
const int kStatPin = 13;

struct RadioMsg
{
  /*0000*/ uint16_t joyX;
  /*0002*/ uint16_t joyY;
};

void setup()
{
#ifdef RH_HAVE_SERIAL
  Serial.begin(115200); // Debugging only
#endif
  if (!driver.init())
#ifdef RH_HAVE_SERIAL
    Serial.println("init failed");
#else
    ;
#endif
}

void loop()
{
  int joyX;
  int joyY;
  const int minX = 0;
  const int minY = 0;
  const int maxX = 1023;
  const int maxY = 1023;
  const int calibrateX = (analogRead(kXPin) - 511);
  const int calibrateY = (analogRead(kYPin) - 511);

  RadioMsg msg;

  int time = 0;
  int state = 0;

  while (state == 0)
  {
    joyX = constrain(map(analogRead(kXPin), minX + calibrateX, maxX + calibrateX, 0, 1023), 0, 1023);
    joyY = constrain(map(analogRead(kYPin), minY + calibrateY, maxY + calibrateY, 0, 1023), 0, 1023);
    msg.joyX = joyX;
    msg.joyY = joyY;

    // -------------------------------------------------
    // Let's send a packet to the receiver.
    uint8_t *data = new uint8_t[sizeof(RadioMsg)];

    memcpy((void *)data, (void *)&msg, sizeof(RadioMsg));

    if (driver.send(data, sizeof(RadioMsg)))
    {
      digitalWrite(kStatPin, HIGH);
    };

    driver.waitPacketSent();
    digitalWrite(kStatPin, LOW);

    delete[] data;

    // ------------------------------------------------
    delay(10);
  }
}