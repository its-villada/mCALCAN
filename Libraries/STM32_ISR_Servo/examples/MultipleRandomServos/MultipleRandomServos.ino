/****************************************************************************************************************************
  MultipleRandomServos.ino
  For STM32F/L/H/G/WB/MP1 boards with stm32duino Arduino_Core_STM32 core
  Written by Khoi Hoang

  Built by Khoi Hoang https://github.com/khoih-prog/STM32_ISR_Servo
  Licensed under MIT license
*****************************************************************************************************************************/

/****************************************************************************************************************************
   From ESP32 Servo Example Using Arduino ESP32 Servo Library
   John K. Bennett
   March, 2017

   Different servos require different pulse widths to vary servo angle, but the range is
   an approximately 500-2500 microsecond pulse every 20ms (50Hz). In general, hobbyist servos
   sweep 180 degrees, so the lowest number in the published range for a particular servo
   represents an angle of 0 degrees, the middle of the range represents 90 degrees, and the top
   of the range represents 180 degrees. So for example, if the range is 1000us to 2000us,
   1000us would equal an angle of 0, 1500us would equal 90 degrees, and 2000us would equal 1800
   degrees.

   - Circuit:
   Servo motors have three wires: power, ground, and signal. The power wire is typically red,
   the ground wire is typically black or brown, and the signal wire is typically yellow,
   orange or white. Since the STM32 can supply limited current at only 3.3V, and servos draw
   considerable power, we will connect servo power to the VBat pin of the STM32 (located
   near the USB connector). THIS IS ONLY APPROPRIATE FOR SMALL SERVOS.

   We could also connect servo power to a separate external power source (as long as we connect all of 
   the grounds (STM32, servo, and external power).
   In this example, we just connect STM32 ground to servo ground. The servo signal pins
   connect to any available GPIO pins on the STM32 (in this example, we use pins (D1-D6).

   In this example, we assume four Tower Pro SG90 small servos.
   The published min and max for this servo are 500 and 2400, respectively.
   These values actually drive the servos a little past 0 and 180, so
   if you are particular, adjust the min and max values to match your needs.
   Experimentally, 800 and 2450 are pretty close to 0 and 180.
*****************************************************************************************************************************/

#if !( defined(STM32F0) || defined(STM32F1) || defined(STM32F2) || defined(STM32F3)  ||defined(STM32F4) || defined(STM32F7) || \
       defined(STM32L0) || defined(STM32L1) || defined(STM32L4) || defined(STM32H7)  ||defined(STM32G0) || defined(STM32G4) || \
       defined(STM32WB) || defined(STM32MP1) || defined(STM32L5))
  #error This code is designed to run on STM32F/L/H/G/WB/MP1 platform! Please check your Tools->Board setting.
#endif

#define TIMER_INTERRUPT_DEBUG       0
#define ISR_SERVO_DEBUG             1

// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
#include "STM32_ISR_Servo.h"

// Default is TIMER_SERVO (TIM7 for many boards)
#define USE_STM32_TIMER_NO          TIMER_SERVO

// Published values for SG90 servos; adjust if needed
#define MIN_MICROS        800  //544
#define MAX_MICROS        2450

#define SERVO_PIN_1       D1
#define SERVO_PIN_2       D2
#define SERVO_PIN_3       D3
#define SERVO_PIN_4       D4
#define SERVO_PIN_5       D5
#define SERVO_PIN_6       D6

typedef struct
{
  int     servoIndex;
  uint8_t servoPin;
} ISR_servo_t;

#if ( defined(STM32L0) || defined(STM32L1) || defined(STM32L4) || defined(STM32L5) )

  #define NUM_SERVOS        3
  
  ISR_servo_t ISR_servo[NUM_SERVOS] =
  {
    { -1, SERVO_PIN_1 }, { -1, SERVO_PIN_2 }, { -1, SERVO_PIN_3 }
  };

#else

  #define NUM_SERVOS        6

  ISR_servo_t ISR_servo[NUM_SERVOS] =
  {
    { -1, SERVO_PIN_1 }, { -1, SERVO_PIN_2 }, { -1, SERVO_PIN_3 }, { -1, SERVO_PIN_4 }, { -1, SERVO_PIN_5 }, { -1, SERVO_PIN_6 }
  };

#endif

void setup()
{
  Serial.begin(115200);
  while (!Serial);

  delay(200);

  Serial.print(F("\nStarting MultipleRandomServos on ")); Serial.println(BOARD_NAME);
  Serial.println(STM32_ISR_SERVO_VERSION);

  //Select STM32 timer USE_STM32_TIMER_NO
  STM32_ISR_Servos.useTimer(USE_STM32_TIMER_NO);

  for (int index = 0; index < NUM_SERVOS; index++)
  {
    ISR_servo[index].servoIndex = STM32_ISR_Servos.setupServo(ISR_servo[index].servoPin, MIN_MICROS, MAX_MICROS);

    if (ISR_servo[index].servoIndex != -1)
    {
      Serial.print(F("Setup OK Servo index = ")); Serial.println(ISR_servo[index].servoIndex);
    }
    else
    {
      Serial.print(F("Setup Failed Servo index = ")); Serial.println(ISR_servo[index].servoIndex);
    }
  }
}

void printServoInfo(int indexServo)
{
  Serial.print(F("Servos idx = "));
  Serial.print(indexServo);
  Serial.print(F(", act. pos. (deg) = "));
  Serial.print(STM32_ISR_Servos.getPosition(ISR_servo[indexServo].servoIndex) );
  Serial.print(F(", pulseWidth (us) = "));
  Serial.println(STM32_ISR_Servos.getPulseWidth(ISR_servo[indexServo].servoIndex));
}

void loop()
{
  int position;      // position in degrees

  position = 0;
  Serial.println(F("Servos @ 0 degree"));
  
  for (int index = 0; index < NUM_SERVOS; index++)
  {
    STM32_ISR_Servos.setPosition(ISR_servo[index].servoIndex, position );
    printServoInfo(index);
  }
  
  // waits 5s between test
  delay(5000);

  position = 90;
  Serial.println(F("Servos @ 90 degree"));
  
  for (int index = 0; index < NUM_SERVOS; index++)
  {
    STM32_ISR_Servos.setPosition(ISR_servo[index].servoIndex, position );
    printServoInfo(index);
  }
  
  // waits 5s between test
  delay(5000);

  position = 180;
  Serial.println(F("Servos @ 180 degree"));
  
  for (int index = 0; index < NUM_SERVOS; index++)
  {
    STM32_ISR_Servos.setPosition(ISR_servo[index].servoIndex, position );
    printServoInfo(index);
  }
  
  // waits 5s between test
  delay(5000);

  Serial.println(F("Servos sweeps from 0-180 degress"));
  
  for (position = 0; position <= 180; position += 1)
  {
    // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    for (int index = 0; index < NUM_SERVOS; index++)
    {
      STM32_ISR_Servos.setPosition(ISR_servo[index].servoIndex, position );
    }
    
    // waits 50ms for the servo to reach the position
    delay(50);
  }
  
  // waits 5s between test
  delay(5000);

  Serial.println(F("Servos sweeps from 180-0 degress"));
  
  for (position = 180; position >= 0; position -= 1)
  {
    // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    for (int index = 0; index < NUM_SERVOS; index++)
    {
      STM32_ISR_Servos.setPosition(ISR_servo[index].servoIndex, position );
    }
    
    // waits 50ms for the servo to reach the position
    delay(50);
  }
  
  // waits 5s between test
  delay(5000);

  Serial.println(F("Servos, index depending, be somewhere from 0-180 degress"));
  
  for (position = 0; position <= 180; position += 1)
  {
    // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    for (int index = 0; index < NUM_SERVOS; index++)
    {
      STM32_ISR_Servos.setPosition(ISR_servo[index].servoIndex, (position + index * (180 / NUM_SERVOS)) % 180 );
    }
    
    // waits 50ms for the servo to reach the position
    delay(50);
  }
  
  delay(5000);

  Serial.println(F("Servos, index depending, be somewhere from 180-0 degress"));
  
  for (position = 180; position >= 0; position -= 1)
  {
    // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    for (int index = 0; index < NUM_SERVOS; index++)
    {
      STM32_ISR_Servos.setPosition(ISR_servo[index].servoIndex, (position + index * (180 / NUM_SERVOS)) % 180 );
    }
    
    // waits 50ms for the servo to reach the position
    delay(50);
  }
  
  // waits 5s between test
  delay(5000);
}
