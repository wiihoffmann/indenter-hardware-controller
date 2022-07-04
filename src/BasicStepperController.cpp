#include "BasicStepperController.h"


BasicStepperController::BasicStepperController(uint8_t stepPin, uint8_t dirPin, bool invertDirPin){
  this->stepPin = stepPin;
  this->dirPin = dirPin;
  stepDelay = 0;
  stepCompensation = 0;
  lastToggleTime = micros();
  lastMoveCallTime = micros();

  if(invertDirPin){
    upPolarity = HIGH;
  }
  else{
    upPolarity = LOW;
  }

  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  digitalWrite(stepPin, LOW);
}


void BasicStepperController::processMove(){
  uint32_t time = micros();
  // if enough time has passed that we should toggle the pin, 
  // and the timeout has not elapsed, and the step delay is non-zero.
  if(time - lastToggleTime >= stepDelay + stepCompensation 
      && time - lastMoveCallTime <= STEPPER_TIMEOUT * 1000 
      && stepDelay != 0){
    
    // calculate the step compensation for the next iteration by
    // averaging this compensation value with the last one.
    stepCompensation = (stepCompensation + (stepDelay - (time - lastToggleTime))) >> 1;
    if(stepCompensation > 0) stepCompensation = 0;

    // flip the pin
    lastToggleTime = time;
    digitalWrite(stepPin, !digitalRead(stepPin));
  }
}


void BasicStepperController::moveUp(uint16_t stepDelay){
  this->stepDelay = stepDelay;
  lastMoveCallTime = micros();
  
  // set dir pin for moving up and move
  digitalWrite(dirPin, upPolarity);
  processMove();
}


void BasicStepperController::moveDown(uint16_t stepDelay){
  this->stepDelay = stepDelay;
  lastMoveCallTime = micros();
  
  // set dir pin for moving down and move
  digitalWrite(dirPin, !upPolarity);
  processMove();
}


void BasicStepperController::stopMoving(){
  stepDelay = 0;
}

