/****************************************************************************************************************************
  STM32_FastTimerInterrupt.h
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

#ifndef STM32_FastTimerInterrupt_h
#define STM32_FastTimerInterrupt_h

#if !( defined(STM32F0) || defined(STM32F1) || defined(STM32F2) || defined(STM32F3)  ||defined(STM32F4) || defined(STM32F7) || \
       defined(STM32L0) || defined(STM32L1) || defined(STM32L4) || defined(STM32H7)  ||defined(STM32G0) || defined(STM32G4) || \
       defined(STM32WB) || defined(STM32MP1) || defined(STM32L5))
  #error This code is designed to run on STM32F/L/H/G/WB/MP1 platform! Please check your Tools->Board setting.
#endif

#include "STM32_ISR_Servo_Debug.h"

// TIMER_NUM defined in cores/arduino/stm32/timer.h
#define MAX_STM32_NUM_TIMERS      TIMER_NUM

class STM32FastTimerInterrupt;

typedef STM32FastTimerInterrupt STM32FastTimer;

typedef void (*stm32_timer_callback)  ();

class STM32FastTimerInterrupt
{
  private:
  
    TIM_TypeDef*    _timer;
    HardwareTimer*  _hwTimer = NULL;
    
    stm32_timer_callback _callback;        // pointer to the callback function
    float             _frequency;       // Timer frequency
    uint64_t          _timerCount;      // count to activate timer

  public:

    STM32FastTimerInterrupt(TIM_TypeDef* timer)
    {              
      _timer = timer;
      
      _hwTimer = new HardwareTimer(_timer);
           
      _frequency  = 0;
      _timerCount = 0;
      _callback = NULL;      
    };
    
    ~STM32FastTimerInterrupt()
    {
      if (_hwTimer)
        delete _hwTimer;
    }

    // frequency (in hertz)
    // No params and duration now. To be added in the future by adding similar functions here or to STM32-hal-timer.c
    bool setFrequency(const float& frequency, stm32_timer_callback callback)
    {
      // select timer frequency is 1MHz for better accuracy. We don't use 16-bit prescaler for now.
      // Will use later if very low frequency is needed.
      _frequency  = 1000000;
      _timerCount = (uint32_t) _frequency / frequency;
      
      ISR_SERVO_LOGERROR1(F("STM32TimerInterrupt: Timer Input Freq (Hz) ="), _hwTimer->getTimerClkFreq());
      ISR_SERVO_LOGERROR3(F("Frequency ="), _frequency, F(", _count ="), (uint32_t) (_timerCount));


      _hwTimer->setCount(0, MICROSEC_FORMAT);
      _hwTimer->setOverflow(_timerCount, MICROSEC_FORMAT);

      _hwTimer->attachInterrupt(callback);
      _hwTimer->resume();

      return true;
    }

    // interval (in microseconds) and duration (in milliseconds). Duration = 0 or not specified => run indefinitely
    // No params and duration now. To be addes in the future by adding similar functions here or to STM32-hal-timer.c
    bool attachInterruptInterval(const unsigned long& interval, stm32_timer_callback callback)
    {
      return setFrequency( (float) ( 1000000.0f / interval), callback);
    }

    void detachInterrupt()
    {
      _hwTimer->detachInterrupt();
    }

    // Duration (in milliseconds). Duration = 0 or not specified => run indefinitely
    void reattachInterrupt()
    {
      setFrequency(_frequency, _callback);
    }
}; // class STM32FastTimerInterrupt


#endif      // STM32_S2_FastTimerInterrupt_h
