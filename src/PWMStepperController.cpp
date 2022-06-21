#include "PWMStepperController.h"
#include <Arduino.h>
#include <TimerOne.h>


int32_t PWMStepperController::displacement;
int8_t PWMStepperController::direction;

PWMStepperController::PWMStepperController(uint8_t stepPin, uint8_t dirPin, bool invertDirPin){
  this->stepPin = stepPin;
  this->dirPin = dirPin;
  this->displacement = 0;
  direction = 0;

  if(invertDirPin){
    upPolarity = HIGH;
  }
  else{
    upPolarity = LOW;
  }

  pinMode(dirPin, OUTPUT);

  // initialize the timer to anything - it gets set to the proper freq later.
  Timer1.initialize(400);
  // increment/decrement the displacement each time the the timer triggers.
  Timer1.attachInterrupt([](){displacement += direction;});
}


void PWMStepperController::startMovingUp(uint16_t stepDelay){
  digitalWrite(dirPin, upPolarity);
  direction = -1;
  Timer1.setPeriod(stepDelay);
  Timer1.pwm(/*pwm pin*/ 9, /*fill factor*/ 512);  // 512 = 50% duty cycle
}


void PWMStepperController::startMovingDown(uint16_t stepDelay){
  digitalWrite(dirPin, !upPolarity);
  direction = 1;
  Timer1.setPeriod(stepDelay);
  Timer1.pwm(/*pwm pin*/ 9, /*fill factor*/ 512);  // 512 = 50% duty cycle
}


void PWMStepperController::stopMoving(){
  Timer1.stop();
  direction = 0;
}


uint32_t PWMStepperController::getDisplacement(){
  return displacement;
}


void PWMStepperController::resetDisplacement(){
  displacement = 0;
}


uint8_t PWMStepperController::getDirection(){
  return direction;
}


void PWMStepperController::emergencyStop(uint16_t stepDelay){
  if(displacement > 0){
    startMovingUp(stepDelay);
    while(displacement > 0);
    stopMoving();
  }
}

