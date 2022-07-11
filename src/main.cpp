#include <Arduino.h>
#include "Communicator.h"
#include "MeasurementController.h"
#include "ADCController.h"
#include "PWMStepperController.h"
#include "BasicStepperController.h"

#define X_STEP_PIN 5
#define X_DIR_PIN 4
#define Y_STEP_PIN 7
#define Y_DIR_PIN 6
#define Z_STEP_PIN 9
#define Z_DIR_PIN 8
#define ADC_INTERRUPT_PIN 2
#define E_STOP_INTERRUPT_PIN 3

Adafruit_ADS1115 ads; /* Use this for the 16-bit version */
// Adafruit_ADS1015 ads;     /* Use this for the 12-bit version */

BasicStepperController *xAxis;
BasicStepperController *yAxis;
BasicStepperController *zAxis;
PWMStepperController *zAxisPWM;

ADCController *adc;
MeasurementController *indenter;
Communicator *comm;

char command;           // the last command we received from the host
uint32_t lastBlink = 0; // last time we blinked the built-in LED
int16_t stepRate;       // how fast to move the stepper


void setup(void){
  pinMode(LED_BUILTIN, OUTPUT);

  // setup serial
  Serial.begin(2000000);
  comm = Communicator::getInstance();

  // set up the motor controllers
  xAxis = new BasicStepperController(X_STEP_PIN, X_DIR_PIN, true);
  yAxis = new BasicStepperController(Y_STEP_PIN, Y_DIR_PIN, true);
  zAxis = new BasicStepperController(Z_STEP_PIN, Z_DIR_PIN, true);
  zAxisPWM = new PWMStepperController(Z_STEP_PIN, Z_DIR_PIN);

  adc = new ADCController(ADC_INTERRUPT_PIN, ads);

  indenter = MeasurementController::getInstance();
  indenter->setUpController(adc, zAxisPWM, E_STOP_INTERRUPT_PIN);

  // tell the host that we are ready to accept commands
  comm->sendCommand(CONTROLLER_READY_CODE);
}


void loop(void){
  command = comm->getCommand();
  
  // if we have a valid command
  if(command != NO_COMMAND_CODE){
    switch(command){
      case EMERGENCY_STOP_CODE: // E-stop. This command should only ever be seen while taking a measurement, not here.
        // TODO: Emergency stop	and send E-stop complete
        comm->sendCommand(EMERGENCY_STOP_CODE, comm->getInt());
        break;
      
      case MOVE_X_AXIS_CODE: // move X axis
        stepRate = comm->getInt();
        // move down if positive, up if negative, stop otherwise
        if(stepRate > 0) xAxis->moveDown(stepRate);
        else if(stepRate < 0) xAxis->moveUp(abs(stepRate));
        else xAxis->stopMoving();
        break;
      
      case MOVE_Y_AXIS_CODE: // move the Y axis
        stepRate = comm->getInt();
        // move down if positive, up if negative, stop otherwise
        if(stepRate > 0) yAxis->moveDown(stepRate);
        else if(stepRate < 0) yAxis->moveUp(abs(stepRate));
        else yAxis->stopMoving();
        break;
      
      case MOVE_Z_AXIS_CODE:	// move the Z axis
        stepRate = comm->getInt();
        // move down if positive, up if negative, stop otherwise
        if(stepRate > 0) zAxis->moveDown(stepRate);
        else if(stepRate < 0) zAxis->moveUp(abs(stepRate));
        else zAxis->stopMoving();
        break;
      
      case BEGIN_MEASUREMENT_CODE: // begin a measurement
        indenter->performMeasurement(comm->receiveMeasurementParams());
        break;
      
      case RAW_MEASUREMENT_CODE: // request for raw measurement
        comm->sendCommand(RAW_MEASUREMENT_CODE, adc->getRawReading());
      
      case ERROR_CODE: // error code
        // TODO: handle error
        break;
      
      default: // unknown command -> error
        comm->sendCommand(ERROR_CODE, 1337);
        break;
    }
  }
  // blink the LED if we are in the main loop waiting for commands
  else if(millis() > lastBlink + 100){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    lastBlink = millis();
  }

  // process any steps the motors may need to make
  xAxis->processMove();
  yAxis->processMove();
  zAxis->processMove();
}



// uint32_t time;
// uint32_t realtime = 0;
// int16_t samples = 0;
// void benchmark(){
//   realtime = (micros() - time);
//   if(realtime >= 1000000){
//     Serial.println();
//     Serial.print("Samples per second: "); Serial.println(samples/(realtime/1000000.0));
//     Serial.print("Samples: "); Serial.println(samples);
//     Serial.print("Real time: "); Serial.println(realtime);
//     Serial.print("Steps: "); Serial.println(zAxisPWM->getDisplacement());

//     samples = 0;
//     realtime = 0;

//     time = micros();
//   }
// }
