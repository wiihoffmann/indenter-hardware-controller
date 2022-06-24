
#include "Arduino.h"
#include "MeasurementController.h"
#include "Communicator.h"


ADCController *MeasurementController::adc;
PWMStepperController *MeasurementController::zAxis;
MeasurementParams MeasurementController::params;
uint8_t MeasurementController::stage;
bool MeasurementController::dataReady;


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


void MeasurementController::performMeasurement(MeasurementParams parameters){
  params = parameters;
  stage = 0;
  dataReady = false;
  Communicator *comm = Communicator::getInstance();

  // Serial.println(params.calFactor);
  // Serial.println(params.preload);
  // Serial.println(params.preloadTime);
  // Serial.println(params.maxLoad);
  // Serial.println(params.maxLoadTime);
  // Serial.println(params.stepDelay);
  // Serial.println(params.holdDownDelay);
  // Serial.println(params.holdUpDelay);

  zAxis->resetDisplacement();
  adc->tare();
  adc->setScaleFactor(params.calFactor);
  adc->startADC([](){dataReady = true;});

  bool doneMeasurement = false;
  uint32_t samples = 0;
  uint32_t start = micros();
  while(!doneMeasurement){
    char command = comm->getCommand();
    if(dataReady){
      dataReady = false;
      samples ++;

      adc->stopADC();
      Serial.println("Stopped!");
      doneMeasurement = true;

      switch(stage){
        case 0: // initial approach
          comm->sendDataPoint(zAxis->getDisplacement(), adc->getLoad(), stage);

          if(zAxis->getDisplacement() >= 1500){ // replace with load stuff
            zAxis->stopMoving();
            stage ++;
            
          }
          else if(zAxis->getDirection() == 0){
            zAxis->startMovingDown(params.stepDelay);
            Serial.println("stage 0");
          }
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
          
          Serial.print("stopping adc... ");
          adc->stopADC();
          Serial.println("Stopped!");
          doneMeasurement = true;
          break;
      }
    }
    else if (command != 'N'){
      Serial.print("Got char: "); Serial.println(command);
      doneMeasurement = true;
      adc->stopADC();
      Serial.println("e-stopping!");
      return;
    }


    // benchmarking function
    if(micros() - start >= 1000000){
      Serial.print("samples last second: "); Serial.println(samples);
      samples = 0;
      start = micros();
    }

  }


}

