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

  // how frequently we expect to see updated on which direction to move the steppers.
  const uint32_t STEPPER_TIMEOUT = 50;

  class BasicStepperController{
    private:
      uint8_t dirPin;             // pin number of the direction output
      uint8_t stepPin;            // pin number of the step signal output
      bool upPolarity;            // flips the polatity of the direction pin
      uint16_t stepDelay;         // how long to wait between moving the stepper by one step
      uint32_t lastToggleTime;    // last time the step pin was flipped (micros)
      int32_t stepCompensation;   // how much to add/subtract from the step delay to achiece the target step delay
      uint32_t lastMoveCallTime;  // last time we received an update on which direction to move the steppers (micros)
      
    public:

      /**
       * Set up a basic stepper controller. This stepper controller bit-bangs
       *  a 50% ducty cycle square wave to drive the stepper.
       * @param stepPin the pin to emit the step signal on
       * @param dirPin the pin to emit the direction signal on
       * @param invertDirPin should the polarity of the direction pin be flipped?
       */
      BasicStepperController(uint8_t stepPin, uint8_t dirPin, bool invertDirPin);
      
      /**
       * Move the stepper up/forward waiting stepDelay*2 between steps.
       * @param stepDelay half of the period to wait between steps.
       */
      void moveUp(uint16_t stepDelay);

      /**
       * Move the stepper udown/backward waiting stepDelay*2 between steps.
       * @param stepDelay half of the period to wait between steps.
       */
      void moveDown(uint16_t stepDelay);
      
      /**
       * stop moving the stepper in any direction.
       */
      void stopMoving();

      /**
       * Service any pending moves (steps) the stepper needs to make. Should be
       * called in the main loop at least as fast as the intended step rate.
       */
      void processMove();
  };

#endif

