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
      uint8_t interruptPin;   // pin the interrupt signal on the ADC connects to
      int16_t offset;         // tare offset
      Adafruit_ADS1X15 *ads;  // the ADC object

      /**
      * Sets the function to be called when the ADC alert pin goes high
      * @param func    A refernce to the function to be called
      */
      void setInterruptFunc(void (*func)());


    public:
      /**
      * Builds a new ADC controller for measuring load applied to the load cell. The load cell is
      * to be connected between analog pins 0 and 1.
      * @param interruptPin The interrupt pin number that the ADC alert line is connected to.
      * @param ads          A reference to the ADS object which initializes the ADC.
      */
      ADCController(uint8_t interruptPin, Adafruit_ADS1115 &ads);

      /**
       * Zero the reading from the ADC 
       */
      void tare();

      /**
       * Gets the reading of the ADC relative to the tare point
       * @return the ADC reading 
       */
      int16_t getLoad();

      /**
       * Tell the ADC to begin performing automated conversions. The alert pin is driven
       * high at the end of each conversion, triggering the interrupt function to run.
       * @param func The function to be run at the completion of each conversion.
       */
      void startADC(void (*func)());
      
      /**
       * Tell the ADC to stop performing automated conversions.
       */
      void stopADC();

      /**
       * Get a raw reading from the ADC, not relative to the tare point.
       * @return the raw (untared) reading from the ADC.
       */
      int16_t getRawReading();


      /**
       * Get a raw voltage reading from the ADC.
       * @return the voltage seen at the ADC in millivolts
       */
      int16_t getVoltageReading();
  };

#endif

