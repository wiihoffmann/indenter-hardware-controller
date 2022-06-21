#include <Arduino.h>
#include "MeasurementController.h"

#ifndef __COMMUNICATOR__
  #define __COMMUNICATOR__
	
	union paramAdapter {
		MeasurementParams parameters;
		byte paramArray[sizeof(MeasurementParams)];
	};

	class Communicator{
	public:
			// delete these methods to prevent making accidental copies of the class.
			Communicator(Communicator const &) = delete;
			void operator=(Communicator const &) = delete;

			static Communicator* getInstance();

			void sendDataPoint(uint32_t displacement, float load, uint8_t measurementStage);
			void sendDataEnd();
			char getCommand();
			MeasurementParams receiveMeasurementParams();


	private:
			Communicator();

	};
#endif
