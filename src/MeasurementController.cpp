#include "MeasurementController.h"
#include "Communicator.h"

// initialize static class variables
ADCController *MeasurementController::adc;
PWMStepperController *MeasurementController::zAxis;
bool MeasurementController::eStop;
bool MeasurementController::doneMeasurement;
volatile bool MeasurementController::dataReady;
uint32_t MeasurementController::holdStartTime;
uint8_t MeasurementController::solenoidPin;
uint8_t MeasurementController::vacuumPin;
uint8_t MeasurementController::VASPin;
uint8_t MeasurementController::indicatorPin;


MeasurementController* MeasurementController::getInstance(){
  static MeasurementController instance; 
  return &instance;
}


MeasurementController::MeasurementController(){
  Serial.flush();
}


void MeasurementController::setUpController(ADCController *adc, PWMStepperController *zAxis, uint8_t eStopInterruptPin, uint8_t solenoidPin, uint8_t vacuumPin, uint8_t VASPin, uint8_t indicatorButtonPin){
  MeasurementController::adc =  adc;
  MeasurementController::zAxis = zAxis;
  MeasurementController::solenoidPin = solenoidPin;
  MeasurementController::vacuumPin = vacuumPin;
  MeasurementController::VASPin = VASPin;
  MeasurementController::indicatorPin = indicatorButtonPin;
  
  pinMode(indicatorPin, INPUT_PULLUP);

  // set up the vacuum pump/gripper
  pinMode(solenoidPin, OUTPUT);
  pinMode(vacuumPin, OUTPUT);
  digitalWrite(solenoidPin, LOW);
  digitalWrite(vacuumPin, LOW);

  // set up the emergency stop interrupt
  pinMode(eStopInterruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(eStopInterruptPin), [](){MeasurementController::eStop = true;}, FALLING);
}


void MeasurementController::emergencyStop(uint16_t stepDelay){
  // terminate measurement by setting flags
  doneMeasurement = true;
  // stop ADC to prevent interrups from it
  adc->stopADC();
  // make sure the vacuum pump is off
  digitalWrite(vacuumPin, LOW);
  // emergency retract the indenter head
  zAxis->emergencyStop(stepDelay);
}


void MeasurementController::applyLoad(int16_t targetLoad, uint16_t stepDelay, int16_t loadActual, uint8_t &stage){
  // move down if actual load is less than the target and we are not already moving down.
  if(zAxis->getDirection() != 1 && loadActual < targetLoad) zAxis->startMovingDown(stepDelay);
  
  // stop moving if the target load is achieved
  if(loadActual >= targetLoad){
    zAxis->stopMoving();
    stage ++;
  }
}


void MeasurementController::holdLoad(int16_t targetLoad, uint16_t tolerance, uint16_t holdDownDelay, uint16_t holdUpDelay, uint16_t holdTime, int16_t loadActual, uint8_t &stage, boolean runVacuum){
  if(holdStartTime == 0){
    holdStartTime = millis();
    // turn on the vacuum 
    if(runVacuum) digitalWrite(vacuumPin, HIGH);
  }

  // if the load is above the upper limit and we are not already moving up -> move up;
  if(loadActual > tolerance + targetLoad && zAxis->getDirection() != 1) zAxis->startMovingUp(holdUpDelay);
  // if the load is below the lower limit and we are not already moving down -> move down;
  else if(loadActual < (double)targetLoad - tolerance && zAxis->getDirection() != -1) zAxis->startMovingDown(holdDownDelay);
  // else we are within the tolerances -> do nothing
  else zAxis->stopMoving();

  // move on to next stage once hold time has elapsed
  if(millis() - holdStartTime >= holdTime){
    zAxis->stopMoving();
    // turn off the vacuum 
    digitalWrite(vacuumPin, LOW);
    holdStartTime = 0;
    stage ++;
  }
}


void MeasurementController::removeLoad(uint16_t stepDelay, uint8_t &stage){
  // if we are not already moving up, and the displacement is still positive, move up
  if(zAxis->getDirection() != -1 && zAxis->getDisplacement() > 0) zAxis->startMovingUp(stepDelay);
  
  // move onto the next stage once the displacement hits zero
  if(zAxis->getDisplacement() <= 0){
    zAxis->stopMoving();
    stage ++;
  }
}


void MeasurementController::runRegularTest(MeasurementParams &params, Communicator *comm){
  uint8_t stage;
  int16_t load;
  char command;
  eStop = false;

  // home the zAxis and start the ADC conversion process
  zAxis->invertDirection(params.flipDirection);
  zAxis->resetDisplacement();
  adc->tare();

  
  for(uint16_t i = 0; i < params.iterations && eStop == false; i++){
    doneMeasurement = false;
    stage = 0;
    holdStartTime = 0;
    if(i > 0)comm->sendCommand(NEW_TEST_BEGIN_CODE);
    adc->startADC([](){dataReady = true;});

    // loop until the measurement completes, or the emergency stop flag is set
    while(!doneMeasurement && !eStop){
      command = comm->getCommand();
      if (command == EMERGENCY_STOP_CODE) eStop = true;
      if (digitalRead(indicatorPin) == LOW) eStop = true;
      
      // if the ADC has signalled that data is available, process it.
      if(dataReady){
        dataReady = false;
        load = adc->getLoad();
        comm->sendDataPoint(zAxis->getDisplacement(), load, stage);      

        // perform the relavent action for the measurement stage we are on
        switch(stage){
          case 0: // initial approach
            applyLoad(params.preload, params.stepDelay, load, stage);
            break;
          case 1: // preload hold
            holdLoad(params.preload, params.tolerance, params.holdDownDelay, params.holdUpDelay, params.preloadTime, load, stage, true);
            break;
          case 2: // main load approach
            applyLoad(params.maxLoad, params.stepDelay, load, stage);     
            break;
          case 3: // main load hold
            holdLoad(params.maxLoad, params.tolerance, params.holdDownDelay, params.holdUpDelay, params.maxLoadTime, load, stage, false);   
            break;
          case 4: // retract
            removeLoad(params.stepDelay, stage);
            break;
          case 5: //done
            adc->stopADC();
            doneMeasurement = true; 
            // turn off the vacuuum and release the gripper
            digitalWrite(vacuumPin, LOW);
            digitalWrite(solenoidPin, HIGH);
            delay(1000);
            digitalWrite(solenoidPin, LOW);
            break;
        }
      }
    }
  }

  // send a final data point with number of samples obtained, and total measurement time (millis)
  // comm->sendDataPoint(samples, millis()-start, 99);
  
  // perform an E-stop if the flag is set
  if(eStop){
    emergencyStop(params.eStopStepDelay);
    // release gripper
    digitalWrite(solenoidPin, HIGH);
    delay(1000);
    digitalWrite(solenoidPin, LOW);
  }

}


void MeasurementController::runPPTTest(MeasurementParams &params, Communicator *comm){
  uint8_t stage;
  int16_t load;
  int16_t maxLoad;
  char command;
  eStop = false;
  boolean indicatorPressed;

  // home the zAxis and start the ADC conversion process
  zAxis->invertDirection(params.flipDirection);
  zAxis->resetDisplacement();
  adc->tare();

  
  for(uint16_t i = 0; i < params.iterations && eStop == false; i++){
    doneMeasurement = false;
    indicatorPressed = false;
    maxLoad = 0;
    stage = 0;
    
    if(i > 0){
      delay(params.preloadTime);
      comm->sendCommand(NEW_TEST_BEGIN_CODE);
    }
    adc->startADC([](){dataReady = true;});

    // loop until the measurement completes, or the emergency stop flag is set
    while(!doneMeasurement && !eStop){
      command = comm->getCommand();
      if (command == EMERGENCY_STOP_CODE) eStop = true;
      if(digitalRead(indicatorPin) == LOW && indicatorPressed == false){
        indicatorPressed = true;
        stage = 1;
      }

      // if the ADC has signalled that data is available, process it.
      if(dataReady){
        dataReady = false;
        load = adc->getLoad();
        comm->sendDataPoint(zAxis->getDisplacement(), load, stage);
        
        if(load > maxLoad) maxLoad = load;  

        // perform the relavent action for the measurement stage we are on
        switch(stage){
          case 0: // main load approach
            applyLoad(params.maxLoad, params.stepDelay, load, stage);     
            break;
          case 1: // retract
            removeLoad(params.stepDelay, stage);
            break;
          case 2: //done
            adc->stopADC();
            doneMeasurement = true; 
            break;
        }
      }
    }
    comm->sendCommand(MAX_LOAD_CODE, maxLoad);

  }

  // send a final data point with number of samples obtained, and total measurement time (millis)
  // comm->sendDataPoint(samples, millis()-start, 99);
  
  // perform an E-stop if the flag is set
  if(eStop){
    emergencyStop(params.eStopStepDelay);
  }
}


void MeasurementController::runPPITest(MeasurementParams &params, Communicator *comm){
  uint32_t delayStart;
  uint8_t stage;
  int16_t load;
  char command;
  eStop = false;

  // home the zAxis and start the ADC conversion process
  zAxis->invertDirection(params.flipDirection);
  zAxis->resetDisplacement();
  adc->tare();

  
  for(uint16_t i = 0; i < params.iterations && eStop == false; i++){
    doneMeasurement = false;
    holdStartTime = 0;
    stage = 0;
    if(i > 0)comm->sendCommand(NEW_TEST_BEGIN_CODE);
    adc->startADC([](){dataReady = true;});

    // loop until the measurement completes, or the emergency stop flag is set
    while(!doneMeasurement && !eStop){
      command = comm->getCommand();
      if (command == EMERGENCY_STOP_CODE) eStop = true;
      if(digitalRead(indicatorPin) == LOW) eStop = true;

      // if the ADC has signalled that data is available, process it.
      if(dataReady){
        dataReady = false;
        load = adc->getLoad();
        comm->sendDataPoint(zAxis->getDisplacement(), load, stage);      

        // perform the relavent action for the measurement stage we are on
        switch(stage){
          case 0: // main load approach
            applyLoad(params.maxLoad, params.stepDelay, load, stage);     
            break;
          case 1: // preload hold
            holdLoad(params.maxLoad, params.tolerance, params.holdDownDelay, params.holdUpDelay, params.maxLoadTime, load, stage, false);
            break;
          case 2: // retract
            removeLoad(params.stepDelay, stage);
            break;
          case 3: //done
            adc->stopADC();
            doneMeasurement = true; 
            break;
        }
      }
    }
    // wait for button press to register VAS score
    while(digitalRead(indicatorPin) && !eStop){
      command = comm->getCommand();
      if (command == EMERGENCY_STOP_CODE) eStop = true;
    }

    // async delay for debounce
    delayStart = millis();
    while(millis() < delayStart + 50){ // 50ms debounce
      command = comm->getCommand();
      if (command == EMERGENCY_STOP_CODE) eStop = true;
    }


    comm->sendCommand(SINGLE_VAS_SCORE_CODE, 1234);


    // wait for button to be released, and wait for debounce
    while(!digitalRead(indicatorPin) && !eStop){
      command = comm->getCommand();
      if (command == EMERGENCY_STOP_CODE) eStop = true;
    };
    
    // async delay for debounce
    delayStart = millis();
    while(millis() < delayStart + 50){ // 50ms debounce
      command = comm->getCommand();
      if (command == EMERGENCY_STOP_CODE) eStop = true;
    }

  }

  // send a final data point with number of samples obtained, and total measurement time (millis)
  // comm->sendDataPoint(samples, millis()-start, 99);
  
  // perform an E-stop if the flag is set
  if(eStop){
    emergencyStop(params.eStopStepDelay);
  }

}


void MeasurementController::runTemporalSummationTest(MeasurementParams &params, Communicator *comm){
  uint8_t stage;
  int16_t load;
  char command;
  eStop = false;

  // home the zAxis and start the ADC conversion process
  zAxis->invertDirection(params.flipDirection);
  zAxis->resetDisplacement();
  adc->tare();
  adc->startADC([](){dataReady = true;});
  
  for(uint16_t i = 0; i < params.iterations && eStop == false; i++){
    doneMeasurement = false;
    stage = 0;
    holdStartTime = 0;    

    // loop until the measurement completes, or the emergency stop flag is set
    while(!doneMeasurement && !eStop){
      command = comm->getCommand();
      if (command == EMERGENCY_STOP_CODE) eStop = true;
      if (digitalRead(indicatorPin) == LOW) eStop = true;
      
      // if the ADC has signalled that data is available, process it.
      if(dataReady){
        dataReady = false;
        load = adc->getLoad();
        
        // TODO: send VAS score here as well!
        comm->sendDataPoint(zAxis->getDisplacement(), load, stage);      

        // perform the relavent action for the measurement stage we are on
        switch(stage){
          case 0: // main load approach
            applyLoad(params.maxLoad, params.stepDelay, load, stage);     
            break;
          case 1: // retract
            removeLoad(params.stepDelay, stage);
            break;
          case 2: //done iteration
            doneMeasurement = true; 
            break;
        }
      }
    }
  }

  adc->stopADC();

  // send a final data point with number of samples obtained, and total measurement time (millis)
  // comm->sendDataPoint(samples, millis()-start, 99);
  
  // perform an E-stop if the flag is set
  if(eStop){
    emergencyStop(params.eStopStepDelay);
  }
}


void MeasurementController::performMeasurement(MeasurementParams params){
  Communicator *comm = Communicator::getInstance();
  
  // switch on the kind of measurement we need to perform
  switch (params.testTypeCode){
    case REGULAR_TEST_CODE:
      runRegularTest(params, comm);
      break;
    case PPT_TEST_CODE:
      runPPTTest(params, comm);
      break;
    case PPI_TEST_CODE:
      runPPITest(params, comm);
      break;
    case TEMPORAL_SUMMATION_TEST_CODE:
      runTemporalSummationTest(params, comm);
      break;
  }
  
  // send the command to denote that the measurement is complete
  comm->sendCommand(MEASUREMENT_COMPLETE_CODE);
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
  // Serial.println(params.flipDirection);




