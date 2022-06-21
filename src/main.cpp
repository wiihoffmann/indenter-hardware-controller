#include <Arduino.h>
#include "Communicator.h"
#include "MeasurementController.h"
#include "ADCController.h"
#include "PWMStepperController.h"
#include "BasicStepperController.h"

Adafruit_ADS1115 ads; /* Use this for the 16-bit version */
// Adafruit_ADS1015 ads;     /* Use this for the 12-bit version */

ADCController *adc;
BasicStepperController *xAxis;
BasicStepperController *yAxis;
BasicStepperController *zAxis;
PWMStepperController *zAxisPWM;

MeasurementController *indenter;
Communicator *comm;


bool ready = false;
double displacement = 0;


uint32_t time;
uint32_t realtime = 0;
int16_t samples = 0;
void benchmark(){
  realtime = (micros() - time);
  if(realtime >= 1000000){
    Serial.println();
    Serial.print("Samples per second: "); Serial.println(samples/(realtime/1000000.0));
    Serial.print("Samples: "); Serial.println(samples);
    Serial.print("Real time: "); Serial.println(realtime);
    Serial.print("Steps: "); Serial.println(zAxisPWM->getDisplacement());

    samples = 0;
    realtime = 0;

    time = micros();
  }
}

        
void setup(void){
  Serial.begin(2000000);

  comm = Communicator::getInstance();

  xAxis = new BasicStepperController(5,4, true);
  yAxis = new BasicStepperController(7,6, true);
  zAxis = new BasicStepperController(9, 8, true);
  zAxisPWM = new PWMStepperController(9, 8, true);

  adc = new ADCController(2, ads);
  adc->setInterruptFunc([](){ready = true;});

  indenter = MeasurementController::getInstance();
  indenter->setUpController();
  
  

  Serial.println("setup complete");
  zAxisPWM->startMovingDown(1200);
}


char command;
void loop(void){

  // if(zAxisPWM->getDisplacement() > 3000) zAxisPWM->startMovingUp(1200);
  // else if(zAxisPWM->getDisplacement() <= 200) zAxisPWM->startMovingDown(1200);


  //Serial.print("Differential: "); Serial.print(results); Serial.print("("); Serial.print(ads.computeVolts(results), 4); Serial.println("V)");

  command = comm->getCommand();
  switch(command){
    case 'E':
      // TODO: Emergency stop	and send e-stop complete
      break;
    case 'X':
      // TODO: move Z axis	
      break;
    case 'Y':
      // TODO: move Z axis	
      break;
    case 'Z':	
      // TODO: move Z axis
      break;
    case 'B':
      Serial.println("case B - starting measurement");
      indenter->performMeasurement(comm->receiveMeasurementParams(), *adc, *zAxisPWM);
      // TODO: begin measurement
      break;
    case 'R':
      // TODO: handle error
      break;
    default:
      // TODO: send an error code here
      break;
  }


}




  // xAxis->moveDown(200);
  // yAxis->moveDown(400);

  // if(ready){
  //   ready = false;
  //   samples ++;
  // }


  // benchmark();