#include <Arduino.h>
#include "Communicator.h"
#include "MeasurementController.h"
#include "ADCController.h"
#include "PWMStepperController.h"
#include "BasicStepperController.h"

Adafruit_ADS1115 ads; /* Use this for the 16-bit version */
// Adafruit_ADS1015 ads;     /* Use this for the 12-bit version */

BasicStepperController *xAxis;
BasicStepperController *yAxis;
BasicStepperController *zAxis;
PWMStepperController *zAxisPWM;

ADCController *adc;
MeasurementController *indenter;
Communicator *comm;

char command;
uint32_t lastBlink =0;
int16_t stepRate;


void setup(void){
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(2000000);
  comm = Communicator::getInstance();

  xAxis = new BasicStepperController(5,4, true);
  yAxis = new BasicStepperController(7,6, true);
  zAxis = new BasicStepperController(9, 8, true);
  zAxisPWM = new PWMStepperController(9, 8);

  adc = new ADCController(2, ads);

  indenter = MeasurementController::getInstance();
  indenter->setUpController(adc, zAxisPWM, 3);

  comm->sendCommand('R');
}



void loop(void){
  command = comm->getCommand();
  
  if(command != 'N'){
    switch(command){
      case 'S': // E-stop. This command should only ever be seen while taking a measurement, not here.
        // TODO: Emergency stop	and send E-stop complete
        comm->sendCommand('S', comm->getInt());
        break;
      
      case 'X': // move X axis
        stepRate = comm->getInt();
        if(stepRate > 0) xAxis->moveDown(stepRate);
        else if(stepRate < 0) xAxis->moveUp(abs(stepRate));
        else xAxis->stopMoving();
        break;
      
      case 'Y': // move the Y axis
        stepRate = comm->getInt();
        if(stepRate > 0) yAxis->moveDown(stepRate);
        else if(stepRate < 0) yAxis->moveUp(abs(stepRate));
        else yAxis->stopMoving();
        break;
      
      case 'Z':	// move the Z axis
        stepRate = comm->getInt();
        if(stepRate > 0) zAxis->moveDown(stepRate);
        else if(stepRate < 0) zAxis->moveUp(abs(stepRate));
        else zAxis->stopMoving();
        break;
      
      case 'B': // begin a measurement
        indenter->performMeasurement(comm->receiveMeasurementParams());
        break;
      
      case 'M': // request for raw measurement
        comm->sendCommand('M', adc->getRawReading());
      
      case 'E': // error code
        // TODO: handle error
        break;
      
      default: // unknown command -> error
        comm->sendCommand('E', 1337);
        break;
    }
  }
  else if(millis() > lastBlink + 100){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    lastBlink = millis();
  }

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
