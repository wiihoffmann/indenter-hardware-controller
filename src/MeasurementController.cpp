
#include "Arduino.h"
#include "MeasurementController.h"
#include "Communicator.h"


ADCController *MeasurementController::adc;
PWMStepperController *MeasurementController::zAxis;
MeasurementParams MeasurementController::params;
uint8_t MeasurementController::stage;
bool MeasurementController::measurementOngoing;


void MeasurementController::DataReadyHandler(){
  switch(stage){
    case 0: // initial approach
      Serial.println("stage 0");
      stage ++;
      break;
    case 1: // preload hold
      Serial.println("stage 1");
      stage ++;
      break;
    case 2: // main load approach
      Serial.println("stage 2");
      stage ++;      
      break;
    case 3: // main load hold
      Serial.println("stage 3");
      stage ++;      
      break;
    case 4: // retract
      Serial.println("stage 4"); 
      measurementOngoing = false;
      Serial.println(measurementOngoing);
      break;
    default:
      // emergency stop
      break;
  }
  return;
}


MeasurementController* MeasurementController::getInstance(){
  static MeasurementController instance; 
  return &instance;
}


MeasurementController::MeasurementController(){
  //measurementOngoing = false;
  Serial.flush();
  Serial.println("ctor");
}


void MeasurementController::setUpController(ADCController *adc, PWMStepperController *zAxis){
  MeasurementController::adc =  adc;
  MeasurementController::zAxis = zAxis;
}


void MeasurementController::emergencyStop(){
  

}

void nothing(){
  delay(1); 
}

void MeasurementController::performMeasurement(MeasurementParams parameters){
  Serial.println("in performMeasurement");
  params = parameters;
  stage = 0;
  measurementOngoing = true;
  //zAxis->resetDisplacement();
  
  adc->startADC(DataReadyHandler);
  // delay(50);
  // adc->stopADC();

  Communicator *comm = Communicator::getInstance();
  while(measurementOngoing != false){
    char command = comm->getCommand();
    if (command == 'S' || command == 'E'){
      // do something here
    }
    delay(1);
  }

  Serial.print("stopping adc... ");
  adc->stopADC();
  Serial.println("Stopped!");

  Serial.println(params.calFactor);
  Serial.println(params.preload);
  Serial.println(params.preloadTime);
  Serial.println(params.maxLoad);
  Serial.println(params.maxLoadTime);
  Serial.println(params.stepDelay);
  Serial.println(params.holdDownDelay);
  Serial.println(params.holdUpDelay);

}

