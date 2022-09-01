/**************************************************************************/
/*!
  @file     Communicator.h

  This is a class to manage the serial communication protocol used by the
	indenter hardware. This class is used to communicate back to the PC 
	side of the operation.
*/
/**************************************************************************/

#ifndef __COMMUNICATOR__
  #define __COMMUNICATOR__
	
  #include <Arduino.h>

  #define ERROR_CODE 'E'
  #define EMERGENCY_STOP_CODE 'S'
  #define MOVE_X_AXIS_CODE 'X'
  #define MOVE_Y_AXIS_CODE 'Y'
  #define MOVE_Z_AXIS_CODE 'Z'
  #define BEGIN_MEASUREMENT_CODE 'B'
  #define REGULAR_DATA_POINT_CODE 'D'
  #define MAX_LOAD_CODE 'b'
  #define DATA_POINT_WITH_VAS_CODE 'V'
  #define SINGLE_VAS_SCORE_CODE 'v'
  #define MEASUREMENT_COMPLETE_CODE 'C'
  #define NEW_TEST_BEGIN_CODE 'N'
  #define RAW_MEASUREMENT_CODE 'M'
  #define CONTROLLER_READY_CODE 'R'
  #define NO_COMMAND_CODE 'K'

  #define REGULAR_TEST_CODE 'R'             // regular test
  #define PPT_TEST_CODE 'T'                 // PPI Test
  #define PPI_TEST_CODE 'I'                 // PPT Test
  #define TEMPORAL_SUMMATION_TEST_CODE 'S'  // temporalSummationTestCode


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
    uint16_t iterations;    // how many interations of the test to run
    bool flipDirection;     // invert the direction of travel for the indenter head
    char testTypeCode;      // character code for the test type to run
  };


  struct DataPoint{
    int32_t displacement; // displacement of the indenter head
    int16_t load;         // load on the indenter head (ADC reading)
    uint8_t stage;        // the measurement stage
  };


  struct DataPointWithVAS{
    int32_t displacement; // displacement of the indenter head
    int16_t load;         // load on the indenter head (ADC reading)
    int16_t VASScore;     // the VAS score
    uint8_t stage;        // the measurement stage
  };


  // union used for serializing the measurement params
	union paramAdapter {
		MeasurementParams parameters;
		byte paramArray[sizeof(MeasurementParams)];
	};


  // union used for serializing data points
	union DataPointAdapter {
		DataPoint point;
		byte pointArray[sizeof(DataPoint)];
	};


  // union used for serializing data points
	union DataPointWithVASAdapter {
		DataPointWithVAS point;
		byte pointArray[sizeof(DataPointWithVAS)];
	};


  // struct DataPointWithButtonState{
  //   int32_t displacement; // displacement of the indenter head
  //   int16_t load;         // load on the indenter head (ADC reading)
  //   boolean buttonState;  // the state of the indicator button
  //   uint8_t stage;        // the measurement stage
  // };

  // // union used for serializing data points
	// union DataPointWithButtonStateAdapter {
	// 	DataPointWithButtonState point;
	// 	byte pointArray[sizeof(DataPointWithButtonState)];
	// };


	class Communicator{
		private:
		  static char validCommands[];
      Communicator();
    
    public:
      // delete these methods to prevent making accidental copies of the class.
      Communicator(Communicator const &) = delete;
      void operator=(Communicator const &) = delete;

      /**
       * Used to get an instance of the communicator singleton.
       * @return a reference to the instance of the communicator class
       */
      static Communicator* getInstance();

      /**
       * Serialize and send a data point over the serial connection.
       * @param displacement the displacement of the indenter head
       * @param load how much load is on the indenter head
       * @param measurementStage the stage the measurement is in
       */
      void sendDataPoint(int32_t displacement, int16_t load, uint8_t measurementStage);
      void sendDataPointWithVAS(int32_t displacement, int16_t load, uint8_t measurementStage, uint16_t VASScore);
      
      
      /**
       * Send the command denoting the end of the measurement data stream.
       */
      void sendDataEnd();
      
      /**
       * Receive a command from the PC host. Nonblocking.
       * @return The command received from the host. 'N' if no command from host.
       */
      char getCommand();

      /**
       * Sends a command and integer pair to the PC host.
       * @param command the command code to send
       * @param data the int to send with the command code
       */
      void sendCommand(char command, int16_t data);

      /**
       * Sends a command code to the PC host.
       * @param command the command code to send
       */      
      void sendCommand(char command);

      /**
       * Receive an int from the serial interface and return it.
       */
      int16_t getInt();

      /**
       * Receive the measurement paramerter struct from the PC.
       * @return measurement parameter struct
       */
      MeasurementParams receiveMeasurementParams();

	};
#endif

