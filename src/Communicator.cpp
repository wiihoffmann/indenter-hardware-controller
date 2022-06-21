#include "Communicator.h"

/*
	Comm protocol:
	*E<int 16>			- ERROR - int gives error code
	*S<int 16> 		  - emergency stop - e-stop delay
	*X<int 16> 			- x-axis move at step delay given by <int 16> - 0=stop - pos/neg sets direction
	*Y<int 16> 			- y-axis move at step delay given by <int 16>
	*Z<int 16>			- z-axis move at step delay given by <int 16>
	*B<double><uint 8><uint8><uint 8><uint 8><uint 16><uint 16><uint 16>				- begin measurement (calibration factor, preload, preload time, max load, max load time, step delay, hold step down delay, hold step up delay)
	*P<uint 32><double><uint 8> - data point (displacement, load, phase)
	*C					- complete

*/

Communicator* Communicator::getInstance(){
  static Communicator instance; // Guaranteed to be destroyed.
                                // Instantiated on first use.
  return &instance;
}


Communicator::Communicator(){
  Serial.println("ctor");
	Serial.flush();
}


void Communicator::sendDataPoint(uint32_t displacement, float load, uint8_t measurementStage){
	uint8_t data[11];
	data[0] = displacement >> 24;
	data[1] = displacement >> 16;
	data[2] = displacement >> 8;
	data[3] = displacement;

	Serial.write(data, 11);
}


void Communicator::sendDataEnd(){
	Serial.write("*C");
}


MeasurementParams Communicator::receiveMeasurementParams(){
	paramAdapter pa;

	while(Serial.available() < sizeof(MeasurementParams)){ 				//TODO: add timeout to waiting
		Serial.print("WAITING. Bytes available: "); Serial.println(Serial.available());
	}
	Serial.readBytes(pa.paramArray, sizeof(MeasurementParams));

	return pa.parameters;
}


char Communicator::getCommand(){
  if(Serial.available() >= 2 && Serial.read() == '*'){
		return Serial.read();
	}
	return 'E';
}

