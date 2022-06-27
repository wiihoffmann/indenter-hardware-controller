#include <Arduino.h>
#include "ADCController.h"
#include "PWMStepperController.h"

#ifndef __MEASUREMENT_CONTROLLER__
  #define __MEASUREMENT_CONTROLLER__


  struct MeasurementParams{
    uint16_t preload;
    uint8_t preloadTime;
    uint16_t maxLoad;
    uint8_t maxLoadTime;
    uint16_t stepDelay;
    uint16_t holdDownDelay;
    uint16_t holdUpDelay;
    uint16_t eStopStepDelay;
    uint16_t tolerance;
  };


  struct DataPoint{
    int32_t displacement;
    int16_t load;
    uint8_t stage;
  };


  class MeasurementController{
    public:
      // delete these methods to prevent making accidental copies of the class.
      MeasurementController(MeasurementController const &) = delete;
      void operator=(MeasurementController const &) = delete;

      static MeasurementController* getInstance();
      static void DataReadyHandler();
      static void setUpController(ADCController *adc, PWMStepperController *zAxis);
      static void performMeasurement(MeasurementParams params);
      static void emergencyStop(uint16_t stepDelay);

    private:
      static ADCController *adc;
      static PWMStepperController *zAxis;
      static bool dataReady;
      static bool doneMeasurement;
      static uint32_t holdStartTime;

      MeasurementController();
      static void applyLoad(uint16_t targetload, uint16_t stepDelay, int16_t loadActual, uint8_t &stage);
      static void holdLoad(uint16_t targetload, uint16_t tolerance, uint16_t holdDownDelay, uint16_t holdUpDelay, uint8_t holdTime, int16_t loadActual, uint8_t &stage);
      static void removeLoad(uint16_t stepDelay, uint8_t &stage);
  };

#endif

