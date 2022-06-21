/**************************************************************************/
/*!
    @file     ADCController.h

    This is a class to interface with the ADC used in the indenter. It will
    setup and read values from the ADC, returning the load in newtons.
*/
/**************************************************************************/

#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include <SPI.h>

#ifndef __ADC_CONTROLLER__
  #define __ADC_CONTROLLER__

  class ADCController{
    private:
      uint8_t interruptPin;
      int16_t reading;
      int16_t offset;
      double scaler;
      Adafruit_ADS1X15 *ads;
      
    public:
      ADCController(uint8_t interruptPin, Adafruit_ADS1115 &ads);
      void setInterruptFunc(void (*func)());
      void tare();
      void setScaleFactor(double scaleFactor);
      double getLoad();
      void startADC(void (*func)());
      void stopADC();
  };

#endif

