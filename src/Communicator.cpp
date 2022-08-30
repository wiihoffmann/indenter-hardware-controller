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
	*N              - New test start
  *C					    - complete
  *M<int 16>      - measure load
  *K              - no command
  *R              - ready
*/

char Communicator::validCommands[] = {ERROR_CODE, EMERGENCY_STOP_CODE, MOVE_X_AXIS_CODE, MOVE_Y_AXIS_CODE, MOVE_Z_AXIS_CODE,
                                      BEGIN_MEASUREMENT_CODE, DATA_POINT_CODE, MEASUREMENT_COMPLETE_CODE, RAW_MEASUREMENT_CODE,
                                      CONTROLLER_READY_CODE,NO_COMMAND_CODE, PEAK_LOAD_CODE, VAS_SCORE_CODE};


Communicator* Communicator::getInstance(){
  static Communicator instance; // Guaranteed to be destroyed.
                                // Instantiated on first use.
  return &instance;
}


Communicator::Communicator(){
	Serial.flush();
}


void Communicator::sendDataPoint(int32_t displacement, int16_t load, uint8_t measurementStage){
  // pack values into data point adapter for serialization
  dataPointAdapter dp;
  dp.point.displacement = displacement;
  dp.point.load = load;
  dp.point.stage = measurementStage;
  
  // write the data point to the serial connection
  Serial.write("*D");
  Serial.write(dp.pointArray, sizeof(DataPoint));
}



void Communicator::sendCommand(char command, int16_t data){
  // serialize the command and int
  uint8_t dataArray[4];
  dataArray[0] = '*';
  dataArray[1] = command;
  dataArray[2] = data;
  dataArray[3] = data >> 8;

  // write the data to the serial port
  Serial.write(dataArray, 4);
}


void Communicator::sendCommand(char command){
  sendCommand(command, 0000);
}


MeasurementParams Communicator::receiveMeasurementParams(){
	paramAdapter pa;
  // wait for all of the data to arrive, and read it into the array part of the union
	while(Serial.available() < sizeof(MeasurementParams)); 				//TODO: add timeout to waiting
	Serial.readBytes(pa.paramArray, sizeof(MeasurementParams));
  
  // return the struct part of the union
	return pa.parameters;
}


int16_t Communicator::getInt(){
  int16_t num = 0;
  // wait for bytes to arrive
	while(Serial.available() < sizeof(uint16_t)); 				//TODO: add timeout to waiting
  
  // read in the int and return it
  num |= (uint16_t)Serial.read();
  num |= (uint16_t)Serial.read() << 8;
  return num;
}


char Communicator::getCommand(){
  char command;
  bool foundAsterisk = false;

  // make sure we have enough bytes to have the asterisk and command letter following it
  if(Serial.available() >= 2){
    // while we have bytes to read, read until we find the asterisk
    while(Serial.available() >= 2 && !foundAsterisk){
      if(Serial.read() == '*') foundAsterisk = true;
    }

    // get the command and make sure it is valid. Return command if valid.
    command = Serial.read();
    for(uint8_t i=0; i < sizeof(validCommands); i++){
      if(validCommands[i] == command){
        return command;
      } 
    }

    // if we found the asterisk and made it here, the command is invalid
    if(foundAsterisk){
      sendCommand(ERROR_CODE, 0001);
    }
  }

  // return 'K' if we did not have enough bytes present to have the asterisk and command
	return NO_COMMAND_CODE;
}

