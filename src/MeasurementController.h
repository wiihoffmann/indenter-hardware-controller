/**************************************************************************/
/*!
    @file     MeasurementController.h

  This class is responsible for performing the stiffness measurement.
*/
/**************************************************************************/


#ifndef __MEASUREMENT_CONTROLLER__
  #define __MEASUREMENT_CONTROLLER__

  #include <Arduino.h>
  #include "ADCController.h"
  #include "PWMStepperController.h"
  #include "Communicator.h"

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

      /**
       * Set up the measurement controller. Initializes the pointers.
       * @param adc a pointer to the ADC to be used
       * @param zAxis a pointer to the zAxis stepper
       * @param eStopInterruptPin the pin the e-stop button is attached to
       */
      static void setUpController(ADCController *adc, PWMStepperController *zAxis, uint8_t eStopInterruptPin, uint8_t solenoidPin, uint8_t vacuumPin);
      
      /**
       * Starts to perform the stiffness measurement
       * @param params the parameters of the measurement
       */
      static void performMeasurement(MeasurementParams params);

    private:
      static ADCController *adc;
      static PWMStepperController *zAxis;
      static bool eStop;                  // flag set to true when performing an e-stop
      static volatile bool dataReady;     // flag set to true when data from ADC is ready
      static bool doneMeasurement;        // flag set to true when the measurement completes
      static uint32_t holdStartTime;      // time (millis) when a hold was initiated
      static uint8_t solenoidPin;
      static uint8_t vacuumPin;

      /**
       * Build a new measurement controller
       */
      MeasurementController();

      /**
       * performs a regular stiffness measurement 
       */
      static void runMeasurementTypeA(MeasurementParams &params, Communicator *comm);

      /**
       * Move the indenter head downward to achieve a given load.
       * @param targetLoad the target load to apply (ADC reading units)
       * @param stepDelay half of the period to wait between steps
       * @param loadActual the current load read by the ADC
       * @param stage the current measurement stage. Increments after load is applied.
       */
      static void applyLoad(int16_t targetLoad, uint16_t stepDelay, int16_t loadActual, uint8_t &stage);
      
      /**
       * Slowly adjusts the indenter head position to maintain a target load
       * @param targetLoad the target load to maintain (ADC reading units)
       * @param tolerace deadband around the target before we start moving the head. (ADC reading units)
       * @param holdDownDelay half of the period to wait between steps when moving downward
       * @param holdUpDelay half of the period to wait between steps when moving upward
       * @param holdTime how long to hold the given load (millis)
       * @param loadActual the actual load reading from the load call (ADC reading units)
       * @param stage the current measurement stage. Increments after load has been held.
       */
      static void holdLoad(int16_t targetLoad, uint16_t tolerance, uint16_t holdDownDelay, uint16_t holdUpDelay, uint16_t holdTime, int16_t loadActual, uint8_t &stage);
      
      /**
       * Retracts the indenter head, removing the applied load.
       * @param stepDelay half of the period to wait between steps.
       * @param stage the current measurement stage. Increments after load has been held.
       */
      static void removeLoad(uint16_t stepDelay, uint8_t &stage);
      
      /**
      * Emergency stop the measurement
      * @param stepDelay half of the time period between steps when performing an e-stop
      */
      static void emergencyStop(uint16_t stepDelay);
  };

#endif

