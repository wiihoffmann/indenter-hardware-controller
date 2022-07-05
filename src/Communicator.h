/**************************************************************************/
/*!
  @file     Communicator.h

  This is a class to manage the serial communication protocol used by the
	indenter hardware. This class is used to communicate back to the PC 
	side of the operation.
*/
/**************************************************************************/

#include <Arduino.h>
#include "MeasurementController.h"

#ifndef __COMMUNICATOR__
  #define __COMMUNICATOR__
	
  #define ERROR_CODE 'E'
  #define EMERGENCY_STOP_CODE 'S'
  #define MOVE_X_AXIS_CODE 'X'
  #define MOVE_Y_AXIS_CODE 'Y'
  #define MOVE_Z_AXIS_CODE 'Z'
  #define BEGIN_MEASUREMENT_CODE 'B'
  #define DATA_POINT_CODE 'D'
  #define MEASUREMENT_COMPLETE_CODE 'C'
  #define RAW_MEASUREMENT_CODE 'M'
  #define CONTROLLER_READY_CODE 'R'
  #define NO_COMMAND_CODE 'N'

  // union used for serializing the measurement params
	union paramAdapter {
		MeasurementParams parameters;
		byte paramArray[sizeof(MeasurementParams)];
	};

  // union used for serializing data points
	union dataPointAdapter {
		DataPoint point;
		byte pointArray[sizeof(DataPoint)];
	};

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

