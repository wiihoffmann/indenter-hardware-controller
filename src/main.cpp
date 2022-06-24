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

        
void setup(void){
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(2000000);
  comm = Communicator::getInstance();

  xAxis = new BasicStepperController(5,4, true);
  yAxis = new BasicStepperController(7,6, true);
  zAxis = new BasicStepperController(9, 8, true);
  zAxisPWM = new PWMStepperController(9, 8, true);

  adc = new ADCController(2, ads);

  indenter = MeasurementController::getInstance();
  indenter->setUpController(adc, zAxisPWM);

  comm->sendCommand('R');
}


uint32_t lastBlink =0;
void loop(void){
  command = comm->getCommand();
  switch(command){
    case 'S':
      // TODO: Emergency stop	and send e-stop complete
      indenter->emergencyStop();
      Serial.print("S: "); Serial.println(comm->getInt());
      break;
    case 'X':
      // TODO: move X axis
      Serial.print("X: "); Serial.println(comm->getInt());
      break;
    case 'Y':
      // TODO: move Y axis	
      Serial.print("Y: "); Serial.println(comm->getInt());
      break;
    case 'Z':	
      // TODO: move Z axis
      Serial.print("Z: "); Serial.println(comm->getInt());
      break;
    case 'B':
      indenter->performMeasurement(comm->receiveMeasurementParams());
      break;
    case 'M': // request for raw measurement
      comm->sendCommand('M', adc->getRawReading());
    case 'E':
      // TODO: handle error
      break;
    default:
      // TODO: send an error code here
      if(millis() > lastBlink + 100){
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        lastBlink = millis();
      }
      break;
  }

}
