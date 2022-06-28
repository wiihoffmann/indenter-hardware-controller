
#include "Arduino.h"
#include "MeasurementController.h"
#include "Communicator.h"


ADCController *MeasurementController::adc;
PWMStepperController *MeasurementController::zAxis;
bool MeasurementController::doneMeasurement;
bool MeasurementController::dataReady;
uint32_t MeasurementController::holdStartTime;


MeasurementController* MeasurementController::getInstance(){
  static MeasurementController instance; 
  return &instance;
}


MeasurementController::MeasurementController(){
  Serial.flush();
}


void MeasurementController::setUpController(ADCController *adc, PWMStepperController *zAxis){
  MeasurementController::adc =  adc;
  MeasurementController::zAxis = zAxis;
}


void MeasurementController::emergencyStop(uint16_t stepDelay){
  doneMeasurement = true;
  adc->stopADC();
  zAxis->emergencyStop(stepDelay);
}


void MeasurementController::applyLoad(int16_t targetload, uint16_t stepDelay, int16_t loadActual, uint8_t &stage){
  if(zAxis->getDirection() != 1 && loadActual < targetload) zAxis->startMovingDown(stepDelay);
  if(loadActual >= targetload){
    zAxis->stopMoving();
    stage ++;
  }
}


void MeasurementController::holdLoad(int16_t targetload, uint16_t tolerance, uint16_t holdDownDelay, uint16_t holdUpDelay, uint16_t holdTime, int16_t loadActual, uint8_t &stage){
  if(holdStartTime == 0) holdStartTime = millis();

  // if the load is above the upper limit and we are not already moving up -> move up;
  if(loadActual > tolerance + targetload && zAxis->getDirection() != 1) zAxis->startMovingUp(holdUpDelay);
  // if the load is below the lower limit and we are not already moving down -> move down;
  else if(loadActual < (double)targetload - tolerance && zAxis->getDirection() != -1) zAxis->startMovingDown(holdDownDelay);
  // else we are within the tolerances -> do nothing
  else zAxis->stopMoving();

  if(millis() - holdStartTime >= holdTime * 1000){
    zAxis->stopMoving();
    holdStartTime = 0;
    stage ++;
  }
}


void MeasurementController::removeLoad(uint16_t stepDelay, uint8_t &stage){
  if(zAxis->getDirection() != -1 && zAxis->getDisplacement() > 0) zAxis->startMovingUp(stepDelay);
  if(zAxis->getDisplacement() <= 0){
    zAxis->stopMoving();
    stage ++;
  }
}


void MeasurementController::performMeasurement(MeasurementParams params){
  Communicator *comm = Communicator::getInstance();
  uint8_t stage = 0;
  double load;
  char command;
  holdStartTime = 0;
  doneMeasurement = false;
  

  zAxis->resetDisplacement();
  adc->tare();
  adc->startADC([](){dataReady = true;});


  uint32_t samples = 0;
  uint32_t start = millis();


  while(!doneMeasurement){
    command = comm->getCommand();
    
    if (command != 'N'){
      adc->stopADC();
      doneMeasurement = true;
      emergencyStop(params.eStopStepDelay);
    }
    
    if(dataReady){
      dataReady = false;
      load = adc->getLoad();
      comm->sendDataPoint(zAxis->getDisplacement(), load, stage);      

      samples ++;

      switch(stage){
        case 0: // initial approach
          applyLoad(params.preload, params.stepDelay, load, stage);
          break;
        case 1: // preload hold
          holdLoad(params.preload, params.tolerance, params.holdDownDelay, params.holdUpDelay, params.preloadTime, load, stage);
          break;
        case 2: // main load approach
          applyLoad(params.maxLoad, params.stepDelay, load, stage);     
          break;
        case 3: // main load hold
          holdLoad(params.maxLoad, params.tolerance, params.holdDownDelay, params.holdUpDelay, params.maxLoadTime, load, stage);   
          break;
        case 4: // retract
          removeLoad(params.stepDelay, stage);
          break;
        case 5: //done
          adc->stopADC();
          doneMeasurement = true;
          break;
      }

      if(millis() - start >= 1000){
        doneMeasurement = true;
      }

    }
  }

  comm->sendDataPoint(samples, millis()-start, 99);
  adc->stopADC();
  comm->sendCommand('C');

  Serial.println(params.preload);
Serial.println(params.preloadTime);
Serial.println(params.maxLoad);
Serial.println(params.maxLoadTime);
Serial.println(params.stepDelay);
Serial.println(params.holdDownDelay);
Serial.println(params.holdUpDelay);
Serial.println(params.eStopStepDelay);
Serial.println(params.tolerance);
}


// Serial.println(params.preload);
// Serial.println(params.preloadTime);
// Serial.println(params.maxLoad);
// Serial.println(params.maxLoadTime);
// Serial.println(params.stepDelay);
// Serial.println(params.holdDownDelay);
// Serial.println(params.holdUpDelay);
// Serial.println(params.eStopStepDelay);
// Serial.println(params.tolerance);




