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
	*C					    - complete
  *N              - no command

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

}


void Communicator::sendDataEnd(){
	Serial.write("*C");
}


MeasurementParams Communicator::receiveMeasurementParams(){
	paramAdapter pa;

	while(Serial.available() < sizeof(MeasurementParams)){ 				//TODO: add timeout to waiting
    Serial.println("waiting for all params to arrive");
	}
	Serial.readBytes(pa.paramArray, sizeof(MeasurementParams));

  Serial.print("remaining bytes available: "); Serial.println(Serial.available());

	return pa.parameters;
}


int16_t Communicator::getInt(){
  int16_t num = 0;
  //wait for bytes to arrive
	while(Serial.available() < sizeof(uint16_t)){ 				//TODO: add timeout to waiting
    Serial.print ("waiting for bytes");
	}
  
  num |= (uint16_t)Serial.read();
  num |= (uint16_t)Serial.read() << 8;
  return num;
}


char Communicator::getCommand(){
  // make sure we have the asterisk and following command letter
  if(Serial.available() >= 2 && Serial.find('*')){
    return Serial.read(); // return the letter after the asterisk
	}

	return 'N';
}

