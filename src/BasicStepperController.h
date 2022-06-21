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

  class BasicStepperController{
    private:
      uint8_t dirPin;
      uint8_t stepPin;
      bool upPolarity;
      uint32_t stepDelay;
      uint32_t lastToggleTime;
      int32_t stepCompensation;
      void move();
      
    public:
      BasicStepperController(uint8_t stepPin, uint8_t dirPin, bool invertDirPin);
      void moveUp(uint32_t stepDelay);
      void moveDown(uint32_t stepDelay);
  };

#endif

