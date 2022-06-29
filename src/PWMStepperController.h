/**************************************************************************/
/*!
    @file     StepperController.h

    This is a class to controll the stepper motors used in the indenter.
    This class relies on PWM and timer 1 to drive the stepper pulse pin.
*/
/**************************************************************************/

#include <Arduino.h>

#ifndef __PWM_STEPPER_CONTROLLER__
  #define __PWM_STEPPER_CONTROLLER__

  class PWMStepperController{
    private:
      uint8_t dirPin;
      uint8_t stepPin;
      bool upPolarity;
      uint16_t stepDelay;
      static volatile int32_t displacement;
      static volatile int8_t direction;
      
    public:
      PWMStepperController(uint8_t stepPin, uint8_t dirPin, bool invertDirPin);
      void startMovingUp(uint16_t stepDelay);
      void startMovingDown(uint16_t stepDelay);
      void stopMoving();
      static int32_t getDisplacement();
      static void resetDisplacement();
      static uint8_t getDirection();
      void emergencyStop(uint16_t stepDelay);
  };

#endif

