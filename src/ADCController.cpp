#include "ADCController.h"


ADCController::ADCController(uint8_t interruptPin, Adafruit_ADS1115 &ads){
  this->interruptPin = interruptPin;
  this->ads = &ads;
  offset = 0;

  // setup interrupt pin
  pinMode(interruptPin, INPUT);

  // halt if we cannot detect the ADC
  if (!this->ads->begin()){
    // blink the built-in LED at .5Hz
    while (1){
      digitalWrite(LED_BUILTIN, HIGH);
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW);
      delay(1000);
    }
  }
  
  // set up the ADC
  this->ads->setGain(GAIN_TWOTHIRDS);             //TODO: change this!
  this->ads->setDataRate(RATE_ADS1115_860SPS);
  stopADC();
}


void ADCController::setInterruptFunc(void (*func)()){
    attachInterrupt(digitalPinToInterrupt(interruptPin), func, RISING);
}


void ADCController::startADC(void (*func)()){
  // wait for any running conversions to complete, and clear it
  while(!ads->conversionComplete());
  ads->getLastConversionResults();
  
  // set the function to run at the end of each conversion, and start converting
  setInterruptFunc(func);
  ads->startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/true);
}


void ADCController::stopADC(){
  // stop converting and remove the interrupt function to prevent spurious calls to it.
  ads->startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/false);
  detachInterrupt(digitalPinToInterrupt(interruptPin));
}


void ADCController::tare(){
  // set the tare offset by averaging 256 readings.
  int32_t sum = 0;
  for (int i = 0; i < 256; i++){
    sum += ads->readADC_Differential_0_1();
  }
  offset = sum / 256;
}


int16_t ADCController::getLoad(){
  // read the adc and tare the reading
  return (ads->getLastConversionResults() - offset);
}


int16_t  ADCController::getRawReading(){
  // raw read the ADC value
  return ads->readADC_Differential_0_1();
}

