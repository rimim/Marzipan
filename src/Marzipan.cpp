// --------------------------------------------------
// Mars motor control
// --------------------------------------------------
// Copyright 2025 Mimir Reynisson
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the “Software”),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <Arduino.h>

/////////////////////////////////////////////////////////////////////

#include "pin-map.h"

#include "pd/Bus.h"
#include "pd/SBus.h"
#include "pd/Log.h"
#include "pd/StreamProxy.h"
#include "pd/CubeMars.h"

// ---------------------------------------------------------------------------
using Bus = pd::Bus;
using AK60 = pd::motor::cubemars::AK60;
// ---------------------------------------------------------------------------

#define MOTOR_ID                1
#define SBUS_THROTTLE_CHANNEL   1
#define TOP_SPEED_LIMIT         0.1f /* Range 0.0 - 1.0 increase for faster response */ 

#define KP                      20.0
#define KD                      1.0
#define TAU                     0.0

// ---------------------------------------------------------------------------
// Error states

enum {
  STATUS_FAILSAFE         = 1<<0,
  STATUS_RESPONDING       = 1<<1,

  STATUS_INITIAL = (
    STATUS_FAILSAFE    |
    STATUS_RESPONDING),
};
unsigned STATUS = STATUS_INITIAL;

// ---------------------------------------------------------------------------
pd::SbusRx sbus_rx(&Serial3);
//pd::SbusTx sbus_tx(&Serial3);
pd::SbusData data;
// ---------------------------------------------------------------------------

// Setup CAN1 stream
pd::FlexCANStreamProxy<CAN2> sCAN1("Can1");

// ---------------------------------------------------------------------------

float mapMinusOneToOne(float x) {
    // x is expected to be in range [-1.0, 1.0]
    // return mapped value in [0.0, 1.0]
    return (x + 1.0f) / 2.0f;  
}

// ---------------------------------------------------------------------------

pd::SBusControllerEvent event;
AK60 motor(MOTOR_ID, "Motor");

bool throttleHasBeenNeutral = false;

bool initMotor() {
  if (!motor.initMotor(AK60::MODE_SPEED)) {
    return false;
  }
  delay(1);
  if (!motor.enableMotor()) {
    return false;
  }
  delay(1);
  return true;
}

void setup(void) {

  Serial.begin(4E6); //4Mbit/s
  // Enable to wait for PC to be ready
  // while (!Serial.dtr()) {
  // }
  pd::platform::init();

  sbus_rx.Begin();

  auto bus = new pd::Bus("CAN", pd::CAN, "Can1", 1000000, 0, 2);

  // Force verbose logging for can commands
  // pd::Log::log().parse("-v:can");

  motor.setBus(bus);
  if (!initMotor()) {
      PDLOG_ERROR("Failed initMotor\n");
  }
}

void printMotorStatus(const char* motorName, const AK60::MotorStatus &motorStatus) {
  if (motorStatus.hasCalibrationError)
    PDLOG_ERROR("ERROR: %s hasCalibrationError\n", motorName);
  if (motorStatus.hasHallEncoderError)
    PDLOG_ERROR("ERROR: %s hasHallEncoderError\n", motorName);
  if (motorStatus.hasMagneticEncodingError)
    PDLOG_ERROR("ERROR: %s hasMagneticEncodingError\n", motorName);
  if (motorStatus.hasOverTemperature)
    PDLOG_ERROR("ERROR: %s hasOverTemperature\n", motorName);
  if (motorStatus.hasOverCurrent)
    PDLOG_ERROR("ERROR: %s hasOverCurrent\n", motorName);
  if (motorStatus.hasUnderVoltage)
    PDLOG_ERROR("ERROR: %s hasUnderVoltage\n", motorName);
}

void loop() {
  if (event.read(sbus_rx) && event.fValid) {
    if (event.fFailSafe) {
      if ((STATUS & STATUS_FAILSAFE) == 0) {
        PDLOG_ERROR("RC FAILSAFE\n");
        STATUS |= STATUS_FAILSAFE;
      }
      // Set everything to neutral in case of failsafe
      event.fValues[SBUS_THROTTLE_CHANNEL] = 0;
    } else {
      STATUS &= ~STATUS_FAILSAFE;
    }
  }

  // Combine user throttle, turn
  // Each of these is in [-1.0, 1.0], so sum can exceed that.
  double throttle = 0.0 - event.fValues[SBUS_THROTTLE_CHANNEL];
  static double lastThrottle = -10;

  // Scale by top speed limit
  throttle *= TOP_SPEED_LIMIT;

  // Clamp final motor commands to [-1.0, 1.0]
  if (throttle >  1.0) throttle =  1.0;
  if (throttle < -1.0) throttle = -1.0;
  if (abs(throttle) < 0.02)
    throttle = 0;

  if (lastThrottle != throttle && motor.control(0, motor.V_MAX * throttle, KP, KD, TAU)) {
    AK60* motors[] = { &motor };
    AK60::process(motors, sizeof(motors)/sizeof(motors[0]));

    // If right motor responded check if there was an error
    if (motor.responded()) {
      auto motorStatus = motor.getStatus();
      if (motor.hasError()) {
        printMotorStatus("motor", motorStatus);
        STATUS &= ~STATUS_RESPONDING;
        lastThrottle = -10;
      } else if ((STATUS & STATUS_RESPONDING) == 0) {
        if (motorStatus.mode != AK60::MODE_SPEED) {
          initMotor();
        } else {
          STATUS |= STATUS_RESPONDING;
          PDLOG_ERROR("RIGHT RESPONDING\n");
          lastThrottle = -10;
        }
      }
    } else if ((STATUS & STATUS_RESPONDING) != 0) {
      PDLOG_ERROR("MOTOR NOT RESPONDING\n");
      STATUS &= ~STATUS_RESPONDING;
      lastThrottle = -10;
    }
  }

}
