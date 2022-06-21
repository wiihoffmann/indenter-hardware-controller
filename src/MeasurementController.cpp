#include "MeasurementController.h"
#include "Arduino.h"


MeasurementController* MeasurementController::getInstance(){
  static MeasurementController instance; 
  return &instance;
}


MeasurementController::MeasurementController(){
  measurementOngoing = false;
  
  Serial.println("ctor");
}


void MeasurementController::setUpController(){

}


void MeasurementController::performMeasurement(MeasurementParams params, ADCController &adc, PWMStepperController &zAxis){
  measurementOngoing = true;
  
  Serial.println("Starting a measurement!");

  
}
