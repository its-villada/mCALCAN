/****************************************************************************************************************************
  STM32_ISR_Servo.hpp
  For STM32F/L/H/G/WB/MP1 boards with stm32duino Arduino_Core_STM32 core
  Written by Khoi Hoang

  Built by Khoi Hoang https://github.com/khoih-prog/STM32_ISR_Servo
  Licensed under MIT license

  Based on SimpleTimer - A timer library for Arduino.
  Author: mromani@ottotecnica.com
  Copyright (c) 2010 OTTOTECNICA Italy

  Based on BlynkTimer.h
  Author: Volodymyr Shymanskyy

  Version: 1.1.0

  Version Modified By   Date      Comments
  ------- -----------  ---------- -----------
  1.0.0   K Hoang      15/08/2021 Initial coding for STM32F/L/H/G/WB/MP1
  1.1.0   K Hoang      06/03/2022 Convert to `h-only` style. Optimize code by using passing by `reference`
 *****************************************************************************************************************************/

#pragma once

#ifndef STM32_ISR_Servo_HPP
#define STM32_ISR_Servo_HPP

#if !defined(STM32_ISR_SERVO_VERSION)
  #define STM32_ISR_SERVO_VERSION             "STM32_ISR_Servo v1.1.0"
  
  #define STM32_ISR_SERVO_VERSION_MAJOR       1
  #define STM32_ISR_SERVO_VERSION_MINOR       1
  #define STM32_ISR_SERVO_VERSION_PATCH       0

  #define STM32_ISR_SERVO_VERSION_INT         1001000
  
#endif

#include <stddef.h>

#include <inttypes.h>

#if defined(ARDUINO)
  #if ARDUINO >= 100
    #include <Arduino.h>
  #else
    #include <WProgram.h>
  #endif
#endif

#include "STM32_ISR_Servo_Debug.h"
#include "STM32_FastTimerInterrupt.h"

#define STM32_MAX_PIN           NUM_DIGITAL_PINS
#define STM32_WRONG_PIN         255

// From Servo.h - Copyright (c) 2009 Michael Margolis.  All right reserved.

#define MIN_PULSE_WIDTH         800       // the shortest pulse sent to a servo  
#define MAX_PULSE_WIDTH         2450      // the longest pulse sent to a servo 
#define DEFAULT_PULSE_WIDTH     1500      // default pulse width when servo is attached
#define REFRESH_INTERVAL        20000     // minumim time to refresh servos in microseconds 

extern void STM32_ISR_Servo_Handler();

class STM32_ISR_Servo
{

  public:
    // maximum number of servos
    const static int MAX_SERVOS = 16;

    // constructor
    STM32_ISR_Servo();

    // destructor
    ~STM32_ISR_Servo()
    {
      if (STM32_ITimer)
      {
        STM32_ITimer->detachInterrupt();
        delete STM32_ITimer;
      }
    }

    void run();

    // useTimer select which timer (0-3) of STM32 to use for Servos
    //Return true if timerN0 in range
    bool useTimer(TIM_TypeDef* timerNo)
    {
      _timerNo = timerNo;
      return true;
    }

    // Bind servo to the timer and pin, return servoIndex
    int8_t setupServo(const uint8_t& pin, const uint16_t& min = MIN_PULSE_WIDTH, const uint16_t& max = MAX_PULSE_WIDTH);

    // setPosition will set servo to position in degrees
    // by using PWM, turn HIGH 'duration' microseconds within REFRESH_INTERVAL (20000us)
    // returns true on success or -1 on wrong servoIndex
    bool setPosition(const uint8_t& servoIndex, const float& position);

    // returns last position in degrees if success, or -1 on wrong servoIndex
    float getPosition(const uint8_t& servoIndex);

    // setPulseWidth will set servo PWM Pulse Width in microseconds, correcponding to certain position in degrees
    // by using PWM, turn HIGH 'pulseWidth' microseconds within REFRESH_INTERVAL (20000us)
    // min and max for each individual servo are enforced
    // returns true on success or -1 on wrong servoIndex
    bool setPulseWidth(const uint8_t& servoIndex, uint16_t& pulseWidth);

    // returns pulseWidth in microsecs (within min/max range) if success, or 0 on wrong servoIndex
    uint16_t getPulseWidth(const uint8_t& servoIndex);

    // destroy the specified servo
    void deleteServo(const uint8_t& servoIndex);

    // returns true if the specified servo is enabled
    bool isEnabled(const uint8_t& servoIndex);

    // enables the specified servo
    bool enable(const uint8_t& servoIndex);

    // disables the specified servo
    bool disable(const uint8_t& servoIndex);

    // enables all servos
    void enableAll();

    // disables all servos
    void disableAll();

    // enables the specified servo if it's currently disabled,
    // and vice-versa
    bool toggle(const uint8_t& servoIndex);

    // returns the number of used servos
    int8_t getNumServos();

    // returns the number of available servos
    int8_t getNumAvailableServos() 
    {
      if (numServos <= 0)
        return MAX_SERVOS;
      else 
        return MAX_SERVOS - numServos;
    };

  private:

    // Use 10 microsecs timer, just fine enough to control Servo, normally requiring pulse width (PWM) 500-2000us in 20ms.
#define TIMER_INTERVAL_MICRO        10

    void init()
    {
      STM32_ITimer = new STM32FastTimer(_timerNo);

      // Interval in microsecs
      if ( STM32_ITimer && STM32_ITimer->attachInterruptInterval(TIMER_INTERVAL_MICRO, (stm32_timer_callback) STM32_ISR_Servo_Handler ) )
      {
        ISR_SERVO_LOGERROR("Starting  ITimer OK");
      }
      else
      {
        ISR_SERVO_LOGERROR("Fail setup STM32_ITimer");      }

      for (uint8_t servoIndex = 0; servoIndex < MAX_SERVOS; servoIndex++)
      {
        memset((void*) &servo[servoIndex], 0, sizeof (servo_t));
        servo[servoIndex].count    = 0;
        servo[servoIndex].enabled  = false;
        // Intentional bad pin
        servo[servoIndex].pin      = STM32_WRONG_PIN;
      }

      numServos   = 0;

      // Init timerCount
      timerCount  = 1;
    }

    // find the first available slot
    int findFirstFreeSlot();

    typedef struct
    {
      uint8_t       pin;                  // pin servo connected to
      unsigned long count;                // In microsecs
      float         position;             // In degrees
      bool          enabled;              // true if enabled
      uint16_t      min;
      uint16_t      max;
    } servo_t;

    volatile servo_t servo[MAX_SERVOS];

    // actual number of servos in use (-1 means uninitialized)
    volatile int8_t numServos;

    // timerCount starts at 1, and counting up to (REFRESH_INTERVAL / TIMER_INTERVAL_MICRO) = (20000 / 10) = 2000
    // then reset to 1. Use this to calculate when to turn ON / OFF pulse to servo
    // For example, servo1 uses pulse width 1000us => turned ON when timerCount = 1, turned OFF when timerCount = 1000 / TIMER_INTERVAL_MICRO = 100
    volatile unsigned long timerCount;

    // For STM32 timer
    TIM_TypeDef*      _timerNo;
    STM32FastTimer*   STM32_ITimer;
};

#endif    // STM32_ISR_Servo_HPP
