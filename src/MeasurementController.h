/**************************************************************************/
/*!
    @file     MeasurementController.h

  This class is responsible for performing the stiffness measurement.
*/
/**************************************************************************/


#include <Arduino.h>
#include "ADCController.h"
#include "PWMStepperController.h"

#ifndef __MEASUREMENT_CONTROLLER__
  #define __MEASUREMENT_CONTROLLER__


  struct MeasurementParams{
    int16_t preload;        // how much preload to apply (target ADC reading)
    uint16_t preloadTime;   // how long to hold the preload (millis)
    int16_t maxLoad;        // the max load to apply (target ADC reading)
    uint16_t maxLoadTime;   // how long to hold the max load (millis)
    uint16_t stepDelay;     // delay between steps (micros)
    uint16_t holdDownDelay; // delay between steps (micros) when moving down to maintain load
    uint16_t holdUpDelay;   // delay between steps (micros) when moving up to maintain load
    uint16_t eStopStepDelay;// delay between steps (micros) when performing an emergency stop
    uint16_t tolerance;     // the hysterisis around the set load point (in raw ADC units)
    bool flipDirection;     // invert the direction of travel for the indenter head
  };


  struct DataPoint{
    int32_t displacement; // displacement of the indenter head
    int16_t load;         // load on the indenter head (ADC reading)
    uint8_t stage;        // the measurement stage
  };


  class MeasurementController{
    public:
      // delete these methods to prevent making accidental copies of the class.
      MeasurementController(MeasurementController const &) = delete;
      void operator=(MeasurementController const &) = delete;

      /**
       * Obtain an instance of the measurement controller singleton.
       * @return a pointer to the measurement controller instance
       */
      static MeasurementController* getInstance();

      
      static void DataReadyHandler();
      static void setUpController(ADCController *adc, PWMStepperController *zAxis, uint8_t eStopInterruptPin);
      static void performMeasurement(MeasurementParams params);
      static void emergencyStop(uint16_t stepDelay);

    private:
      static ADCController *adc;
      static PWMStepperController *zAxis;
      static bool eStop;
      static volatile bool dataReady;
      static bool doneMeasurement;
      static uint32_t holdStartTime;

      MeasurementController();
      static void applyLoad(int16_t targetload, uint16_t stepDelay, int16_t loadActual, uint8_t &stage);
      static void holdLoad(int16_t targetload, uint16_t tolerance, uint16_t holdDownDelay, uint16_t holdUpDelay, uint16_t holdTime, int16_t loadActual, uint8_t &stage);
      static void removeLoad(uint16_t stepDelay, uint8_t &stage);
  };

#endif

