#include "PWMStepperController.h"
#include <Arduino.h>
#include <TimerOne.h>


volatile int32_t PWMStepperController::displacement;
volatile int8_t PWMStepperController::direction;

PWMStepperController::PWMStepperController(uint8_t stepPin, uint8_t dirPin){
  this->stepPin = stepPin;
  this->dirPin = dirPin;
  this->displacement = 0;
  direction = 0;
  upPolarity = HIGH;

  pinMode(dirPin, OUTPUT);

  // initialize the timer to anything - it gets set to the proper freq later.
  Timer1.initialize(400);
  // increment/decrement the displacement each time the the timer triggers.
  Timer1.attachInterrupt([](){displacement += direction;});
}


void PWMStepperController::invertDirection(){
  upPolarity = !upPolarity;
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


int32_t PWMStepperController::getDisplacement(){
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

