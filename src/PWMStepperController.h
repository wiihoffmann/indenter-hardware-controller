/**************************************************************************/
/*!
    @file     StepperController.h

    This is a class to controll the stepper motors used in the indenter.
    This class relies on PWM and timerOne to drive the stepper pulse pin.
*/
/**************************************************************************/

#include <Arduino.h>
#include <TimerOne.h>

#ifndef __PWM_STEPPER_CONTROLLER__
  #define __PWM_STEPPER_CONTROLLER__

  class PWMStepperController{
    private:
      uint8_t dirPin;                       // direction signal output pin number
      uint8_t stepPin;                      // step output pin number
      bool upPolarity;                      // flip direction corresponding to up
      uint16_t stepDelay;                   // the period between steps
      static volatile int32_t displacement; // how far the indenter head has moved
      static volatile int8_t direction;     // current direction the head is moving
      
    public:

      /**
       * Build a new PWM-based stepper controller.
       * @param stepPin the PWM pin to use (must be on timer one)
       * @param dirPin the pin to emit the direction signal on
       */
      PWMStepperController(uint8_t stepPin, uint8_t dirPin);
      
      /**
       * inverts the polarity of the step pin. Call if the 
       * indenter moves up when it should move down or vise versa.
       */
      void invertDirection(bool flip);

      /**
       * Call to start moving the stepper in the upward direction.
       * @param stepDelay the period between steps.
       */
      void startMovingUp(uint16_t stepDelay);

      /**
       * Call to start moving the stepper in the upward direction.
       * @param stepDelay the period between steps.
       */
      void startMovingDown(uint16_t stepDelay);

      /**
       * Call to stop moving the stepper.
       */
      void stopMoving();

      /**
       * Returns the total displacement of the stepper from its starting position.
       * @return the displacement in steps of the stepper from its initial position.
       */
      static int32_t getDisplacement();

      /**
       * Zeros the displacement of the stepper motor.
       */
      static void resetDisplacement();

      /**
       * Returns the current direction the stepper is moving in.
       * @return -1 for upward, 1 for downward, 0 for stopped.
       */
      static uint8_t getDirection();

      /**
       * Returns the stepper back to the zero displacement location.
       * @param stepDelay the period between steps.
       */
      void emergencyStop(uint16_t stepDelay);
  };

#endif

