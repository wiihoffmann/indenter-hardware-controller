/**************************************************************************/
/*!
    @file     StepperController.h

    This is a class to controll the stepper motors used in the indenter.
    This class relies on delays to drive the stepper pulse pin.
*/
/**************************************************************************/

#include <Arduino.h>

#ifndef __BASIC_STEPPER_CONTROLLER__
  #define __BASIC_STEPPER_CONTROLLER__

  #define STEPPER_TIMEOUT 50

  class BasicStepperController{
    private:
      uint8_t dirPin;
      uint8_t stepPin;
      bool upPolarity;
      uint16_t stepDelay;
      uint32_t lastToggleTime;
      int32_t stepCompensation;
      uint32_t lastMoveCallTime;
      
      
    public:
      BasicStepperController(uint8_t stepPin, uint8_t dirPin, bool invertDirPin);
      void moveUp(uint16_t stepDelay);
      void moveDown(uint16_t stepDelay);
      void stopMoving();
      void processMove();
  };

#endif

