#include "BasicStepperController.h"
#include <Arduino.h>


BasicStepperController::BasicStepperController(uint8_t stepPin, uint8_t dirPin, bool invertDirPin){
  this->stepPin = stepPin;
  this->dirPin = dirPin;
  stepCompensation = 0;
  lastToggleTime = micros();

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


void BasicStepperController::move(){
  uint32_t time = micros();
  
  if(time - lastToggleTime >= stepDelay + stepCompensation){
    stepCompensation = (stepCompensation + (stepDelay - (time - lastToggleTime))) >> 1;
    if(stepCompensation > 0) stepCompensation = 0;

    lastToggleTime = time;
    digitalWrite(stepPin, !digitalRead(stepPin));
  }
}


void BasicStepperController::moveUp(uint32_t stepDelay){
  this->stepDelay = stepDelay;
  digitalWrite(dirPin, upPolarity);
  move();
}


void BasicStepperController::moveDown(uint32_t stepDelay){
  this->stepDelay = stepDelay;
  digitalWrite(dirPin, !upPolarity);
  move();
}

