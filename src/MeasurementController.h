#include <Arduino.h>
#include "ADCController.h"
#include "PWMStepperController.h"

#ifndef __MEASUREMENT_CONTROLLER__
  #define __MEASUREMENT_CONTROLLER__
  
  struct MeasurementParams{
    double calFactor;
    uint8_t preload;
    uint8_t preloadTime;
    uint8_t maxLoad;
    uint8_t maxLoadTime;
    uint16_t stepDelay;
    uint16_t holdDownDelay;
    uint16_t holdUpDelay;
  };
  
  class MeasurementController{
    public:
        // delete these methods to prevent making accidental copies of the class.
        MeasurementController(MeasurementController const &) = delete;
        void operator=(MeasurementController const &) = delete;

        static MeasurementController* getInstance();
        void setUpController();

        void performMeasurement(MeasurementParams params, ADCController &adc, PWMStepperController &zAxis);


    private:
        

        bool measurementOngoing;

        MeasurementController();


  };

#endif