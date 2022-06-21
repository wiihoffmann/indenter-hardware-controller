#include "ADCController.h"
#include <Arduino.h>


ADCController::ADCController(uint8_t interruptPin, Adafruit_ADS1115 &ads){
  this->interruptPin = interruptPin;
  this->ads = &ads;

  // setup interrupt pin
  pinMode(interruptPin, INPUT);

  // halt if we cannot detect the ADC
  if (!this->ads->begin()){
    while (1){
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    }
  }
  
  this->ads->setGain(GAIN_TWOTHIRDS);                          //TODO: change this!
  this->ads->setDataRate(RATE_ADS1115_860SPS);
  
  // make sure the ADC is not running
  stopADC();

  scaler = 0; //TODO: code to calculate/set the scaler
}


void ADCController::setInterruptFunc(void (*func)()){
    attachInterrupt(digitalPinToInterrupt(interruptPin), func, RISING);
}


void ADCController::startADC(void (*func)()){
  while(!ads->conversionComplete());
  ads->getLastConversionResults();
  setInterruptFunc(func);
  ads->startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/true);
}


void ADCController::stopADC(){
  ads->startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/false);
  detachInterrupt(digitalPinToInterrupt(interruptPin));
}


void ADCController::tare(){
  int32_t sum = 0;
  int readings = 0;
  for (int i = 0; i < 256; i++){
    sum += ads->readADC_Differential_0_1();
    readings ++;
  }
  offset = sum / 256;
}


void ADCController::setScaleFactor(double scaleFactor){
  scaler = scaleFactor;
}


double ADCController::getLoad(){
  return (ads->getLastConversionResults() - offset) * scaler;
}
