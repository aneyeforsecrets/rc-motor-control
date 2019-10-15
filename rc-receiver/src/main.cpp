#include <Arduino.h>
#include <LiquidCrystal_PCF8574.h>
#include <Wire.h>

// ask_receiver.pde
// -*- mode: C++ -*-
// Simple example of how to use RadioHead to receive messages
// with a simple ASK transmitter in a very simple way.
// Implements a simplex (one-way) receiver with an Rx-B1 module
// Tested on Arduino Mega, Duemilanova, Uno, Due, Teensy, ESP-12
#include <RH_ASK.h>
#ifdef RH_HAVE_HARDWARE_SPI
#include <SPI.h> // Not actually used but needed to compile
#endif
RH_ASK driver;
// RH_ASK driver(2000, 4, 5, 0); // ESP8266 or ESP32: do not use pin 11 or 2
// RH_ASK driver(2000, 3, 4, 0); // ATTiny, RX on D3 (pin 2 on attiny85) TX on D4 (pin 3 on attiny85),

struct RadioMsg
{
  /*0000*/ uint16_t joyX;
  /*0002*/ uint16_t joyY;
};

enum MotorMode
{
  BRAKEVCC,
  CW,
  CCW,
  BRAKEGND,
};

enum Motor
{
  MOTOR_A,
  MOTOR_B,
};

const uint8_t PWM_MAX = 255;
const uint8_t PWM_HALF = PWM_MAX / 2;

const int kCurrentSensingThreshhold = 5000;

/* Voltage controlled input pins with hysteresis, CMOS compatible. These two pins
control the state of the bridge in normal operation according to the truth table (brake
to VCC , brake to GND, clockwise and counterclockwise).

Table 1.    Truth table in normal operating conditions
+------+-----+------+-----+-------+-------+---------------------+------------------------+
| INA  | INB | ENA  | ENB | OUTA  | OUTB  |         CS          |          MODE          |
+------+-----+------+-----+-------+-------+---------------------+------------------------+
|    1 |   1 |    1 |   1 | H     | H     | High Imp.           | Brake to VCC           |
|    1 |   0 |    1 |   1 | H     | L     | I_Sense = I_OUT / K | Clockwise (CW)         |
|    0 |   1 |    1 |   1 | L     | H     | I_SENSE = I_OUT / K | Counterclockwise (CCW) |
|    0 |   0 |    1 |   1 | L     | L     | High Imp.           | Brake to GND           |
+------+-----+------+-----+-------+-------+---------------------+------------------------+
For more information, see the VNH2SP30-E motor driver datasheet */
const int kInAPin[2] = {2, 0};
const int kInBPin[2] = {3, 0};

/* Voltage controlled input pins with hysteresis, CMOS compatible. Gates of low side
FETs are modulated by the PWM signal during their ON phase allowing speed
control of the motors. */
const int kPwmPin[2] = {5, 0};

/* When pulled low, disables the half-bridge of the VNH2SP30-E of the motor. In case of 
fault detection (thermal shutdown of a high side FET or excessive ON state voltage drop 
across a low side FET), this pin is pulled low by the device. 

Table 2.    Truth table in fault conditions (detected on OUTA)
+------+-----+------+-----+-------+-------+------------+
| INA  | INB | ENA  | ENB | OUTA  | OUTB  |     CS     |
+------+-----+------+-----+-------+-------+------------+
| 1    | 1   |    1 |   1 | OPEN  | H     | High Imp.  |
| 1    | 0   |    1 |   1 | OPEN  | L     | High Imp.  |
| 0    | 1   |    1 |   1 | OPEN  | H     | I_OUTB / K |
| 0    | 0   |    1 |   1 | OPEN  | L     | High Imp.  |
| X    | X   |    0 |   0 | OPEN  | OPEN  | High Imp.  |
| X    | 1   |    0 |   1 | OPEN  | H     | I_OUTB / K |
| X    | 0   |    0 |   1 | OPEN  | L     | High Imp.  |
+------+-----+------+-----+-------+-------+------------+
For more information, see the VNH2SP30-E motor driver datasheet */
const int kEnPin[2] = {1, 0};

/* Analog current sense input. This input senses a current proportional to the motor
current. The information can be read back as an analog voltage. */
const int kCspPin[2] = {0, 0};

// On-Board LED used as status indication
const int kStatPin = 13;

// LCD
const int kLcdAddress = 0x27;
LiquidCrystal_PCF8574 lcd(kLcdAddress);

void lcdSetup(int address)
{
  Wire.beginTransmission(kLcdAddress);
  lcd.begin(16, 2);
  lcd.setBacklight(255);
}

void motorSetup()
{
  pinMode(kStatPin, OUTPUT);

  // Initialize digital pins as outputs
  for (int i = 0; i < 2; i++)
  {
    pinMode(kInAPin[i], OUTPUT);
    pinMode(kInBPin[i], OUTPUT);
    pinMode(kPwmPin[i], OUTPUT);
  }
  // Initialize braked
  for (int i = 0; i < 2; i++)
  {
    digitalWrite(kInAPin[i], LOW);
    digitalWrite(kInBPin[i], LOW);
  }
}
void motorOff(int motor)
{
  // Initialize braked
  for (int i = 0; i < 2; i++)
  {
    digitalWrite(kInAPin[i], LOW);
    digitalWrite(kInBPin[i], LOW);
  }
  analogWrite(kPwmPin[motor], 0);
}

/* motorGo() will set a motor going in a specific direction
 the motor will continue going in that direction, at that speed
 until told to do otherwise.
 
 motor: this should be either MOTOR_A (0) or MOTOR_B (1), will select 
 which of the two motors to be controlled. Invalid values are ignored.
 
 mode: Should be one of the following values
  BRAKEVCC (0): Brake to VCC
  CW (1): Turn Clockwise
  CCW (2): Turn Counter-Clockwise
  BRAKEGND (3): Brake to GND
 
 speed: should be a value between 0 and PWM_MAX (255), higher the number, the faster
 */
void motorGo(Motor motor, MotorMode mode, uint8_t speed)
{

  if (motor == MOTOR_A || motor == MOTOR_B)
  {
    switch (mode)
    {
    case BRAKEVCC: // Brake to VCC
      digitalWrite(kInAPin[motor], HIGH);
      digitalWrite(kInBPin[motor], HIGH);
      break;
    case CW: // Turn Clockwise
      digitalWrite(kInAPin[motor], HIGH);
      digitalWrite(kInBPin[motor], LOW);
      break;
    case CCW: // Turn Counter-Clockwise
      digitalWrite(kInAPin[motor], LOW);
      digitalWrite(kInBPin[motor], HIGH);
      break;
    case BRAKEGND: // Brake to GND
      digitalWrite(kInAPin[motor], LOW);
      digitalWrite(kInBPin[motor], LOW);
      break;

    default:
      // Invalid mode does not change the PWM signal
      return;
    }
    analogWrite(kPwmPin[motor], speed);
  }
  return;
}

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

  Wire.begin();

  motorSetup();
  delay(100);
  lcdSetup(kLcdAddress);
  delay(100);
} // setup()

void loop()
{
  int joyX = 511;
  int joyY = 511;
  const int deadZoneX = 5;
  const int deadZoneY = 5;

  uint8_t speed;
  MotorMode mode;
  int time = 0;

  String line0;
  String line1;

  float motorCurrent;
  int state = 0;

  while (state == 0)
  {
    uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
    uint8_t buflen = sizeof(buf);

    if (driver.recv(buf, &buflen)) // Non-blocking
    {
      digitalWrite(kStatPin, HIGH);
      RadioMsg msg;
      memcpy(&msg, buf, sizeof(RadioMsg)); // Fill the structure with the incoming data.
                                           // Message with a good checksum received, dump it.
      joyX = msg.joyX;
      joyY = msg.joyY;
    }

    digitalWrite(kStatPin, LOW);

    speed = map(abs(joyX - 511),0,511,0,255);

    if (joyX - 511 < -deadZoneX)
    {
      mode = CCW;
    }
    else if (joyX - 511 > deadZoneX)
    {
      mode = CW;
    }
    else
    {
      mode = BRAKEVCC;
      speed = 0;
    }

    if (time % 200 == 0)
    {
      line0 = String("");
      line1 = String("");
      line0 = String(line0 + "Speed: " + String(speed));
      line1 = String(line1 + "JoyX: " + String(joyX));
      lcd.clear();
      
      lcd.print(line0);
      lcd.setCursor(0, 1);
      lcd.print(line1);
      time = 0;
    }

    motorGo(MOTOR_A, mode, speed);

    // if ((analogRead(kCspPin[0]) < kCurrentSensingThreshhold) && (analogRead(kCspPin[1]) < kCurrentSensingThreshhold))
    // {
    //   digitalWrite(kStatPin, HIGH);
    // }
    // else
    // {
    //   digitalWrite(kStatPin, LOW);
    //   break;
    // }
    time = time + 10;
    delay(10);
  }
  motorOff(MOTOR_A);
}