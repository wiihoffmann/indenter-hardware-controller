#include "Communicator.h"

/*
	Comm protocol:
	*E<int 16>			- ERROR - int gives error code
	*S<int 16> 		  - emergency stop - e-stop delay
	*X<int 16> 			- x-axis move at step delay given by <int 16> - 0=stop - pos/neg sets direction
	*Y<int 16> 			- y-axis move at step delay given by <int 16>
	*Z<int 16>			- z-axis move at step delay given by <int 16>
	*B<int 16><uint16><int 16><uint 16><uint 16><uint 16><uint 16><uint 16><int 16>				- begin measurement (preload, preload time, max load, max load time, step delay, hold step down delay, hold step up delay, e-stop step delay, hold load tolerance)
	*D<int 32><double><uint 8> - data point (displacement, load, phase)
	*C					    - complete
  *M<int 16>      - measure load
  *N              - no command

*/

Communicator* Communicator::getInstance(){
  static Communicator instance; // Guaranteed to be destroyed.
                                // Instantiated on first use.
  return &instance;
}


Communicator::Communicator(){
	Serial.flush();
}


void Communicator::sendDataPoint(int32_t displacement, int16_t load, uint8_t measurementStage){
  dataPointAdapter dp;
  dp.point.displacement = displacement;
  dp.point.load = load;
  dp.point.stage = measurementStage;
  
  Serial.write("*D");
  Serial.write(dp.pointArray, sizeof(DataPoint));
}



void Communicator::sendCommand(char command, int16_t data){
  uint8_t dataArray[4];
  dataArray[0] = '*';
  dataArray[1] = command;
  dataArray[2] = data;
  dataArray[3] = data >> 8;
  Serial.write(dataArray, 4);
}


void Communicator::sendCommand(char command){
  sendCommand(command, 0000);
}


MeasurementParams Communicator::receiveMeasurementParams(){
	paramAdapter pa;

	while(Serial.available() < sizeof(MeasurementParams)); 				//TODO: add timeout to waiting
	Serial.readBytes(pa.paramArray, sizeof(MeasurementParams));

	return pa.parameters;
}


int16_t Communicator::getInt(){
  int16_t num = 0;
  //wait for bytes to arrive
	while(Serial.available() < sizeof(uint16_t)){ 				//TODO: add timeout to waiting
    delay(1);
	}
  
  num |= (uint16_t)Serial.read();
  num |= (uint16_t)Serial.read() << 8;
  return num;
}


char Communicator::getCommand(){
  // make sure we have the asterisk and following command letter
  if(Serial.available()>=2){
    while(Serial.available() >= 2 && Serial.read() != '*'); 
    return Serial.read();
  }

	return 'N';
}

