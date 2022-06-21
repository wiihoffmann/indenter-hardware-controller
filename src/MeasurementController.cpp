
#include "Arduino.h"
#include "MeasurementController.h"


ADCController *MeasurementController::adc;
PWMStepperController *MeasurementController::zAxis;
MeasurementParams MeasurementController::params;
uint8_t MeasurementController::stage;
bool MeasurementController::measurementOngoing;


void MeasurementController::DataReadyHandler(){
  switch(stage){
    case 0: // initial approach
      Serial.println("stage 0");
      stage = 1;
      break;
    case 1: // preload hold
            Serial.println("stage 1");
      stage = 2;
      break;
    case 2: // main load approach
      Serial.println("stage 2");
      stage = 3;      
      break;
    case 3: // main load hold
      Serial.println("stage 3");
      stage = 4;      
      break;
    case 4: // retract
      Serial.println("stage 4"); 
      adc->stopADC();  
      break;
    default:
      // emergency stop
      break;
  }
  if(stage == 4){
    Serial.println("in interrupt func");
  }
}


MeasurementController* MeasurementController::getInstance(){
  static MeasurementController instance; 
  return &instance;
}


MeasurementController::MeasurementController(){
  //measurementOngoing = false;
  
  Serial.println("ctor");
}


void MeasurementController::setUpController(ADCController *adc, PWMStepperController *zAxis){
  MeasurementController::adc =  adc;
  // indenterHead = NULL;
}


void MeasurementController::emergencyStop(){
  

}

void nothing(){
  delay(100); 
  //MeasurementController::adc->stopADC();
}

void MeasurementController::performMeasurement(MeasurementParams parameters){
  Serial.println("in performMeasurement");
  params = parameters;
  stage = 0;
  MeasurementController::zAxis->resetDisplacement();
  
  adc->startADC(nothing);
  Serial.println("adc started");
  adc->stopADC();

  measurementOngoing = true;
  
  Serial.println(params.calFactor);
  Serial.println(params.preload);
  Serial.println(params.preloadTime);
  Serial.println(params.maxLoad);
  Serial.println(params.maxLoadTime);
  Serial.println(params.stepDelay);
  Serial.println(params.holdDownDelay);
  Serial.println(params.holdUpDelay);

  
}

