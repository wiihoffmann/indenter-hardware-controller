from dataclasses import dataclass
import time
import serial
import serial.tools.list_ports
from struct import *



@dataclass
class MeasurementParams:
    preamble = bytes("*B", 'utf-8') #     start measurement
    calFactor = 1                   #     double calFactor;
    preload = 300                   #     int16_t preload;
    preloadTime = 4000              #     uint16_t preloadTime;
    maxLoad = 1000                  #     int16_t maxLoad;
    maxLoadTime = 4000              #     uint16_t maxLoadTime;
    stepDelay = 700                 #     uint16_t stepDelay;
    holdDownDelay = 700             #     uint16_t holdDownDelay;
    holdUpDelay = 800               #     uint16_t holdUpDelay;
    eStopStepDelay = 700            #     uint16_t eStopStepDelay;
    tolerance = 100                 #     uint16_t targetTolerance;
    flipDirection = True            #     bool flip indenter direction

class Communicator:

    def __init__(self):
        ports = list(serial.tools.list_ports.comports())
        self.arduino = serial.Serial(port=ports.pop().device, baudrate=2000000, timeout=None)
        self.arduino.flush()


    def sendMeasurementBegin(self, params):
        dataToSend = pack("<%dshHhHHHHHH?" % (len(params.preamble)), params.preamble, params.preload, params.preloadTime, params.maxLoad, params.maxLoadTime, params.stepDelay, params.holdDownDelay, params.holdUpDelay, params.eStopStepDelay, params.tolerance, params.flipDirection)
        self.arduino.write(dataToSend)


    def sendCode(self, preamble, int=0000):
        preamble = bytes(preamble, 'utf-8')
        dataToSend = pack("<%dsh" % (len(preamble)), preamble, int)
        self.arduino.write(dataToSend)


    def commandAvailable(self):
        return self.arduino.in_waiting >= 4


    def readCommand(self):
        while not self.commandAvailable():
            time.sleep(0)
        
        numBytesWaiting = self.arduino.in_waiting
        readBytes = self.arduino.read_until(b'*', numBytesWaiting)
        if len(readBytes) < numBytesWaiting:
            return self.arduino.read().decode('utf-8')
        
        print("ERROR: trying to read command again!")
        print("Discarded bytes: " + str(readBytes))
        return(self.readCommand())


    def readInt(self):
        data = self.arduino.read(2)
        value = int.from_bytes(data, byteorder='little', signed=True)
        return value


    def readDataPoint(self):
        # wait for the data point to arrive
        while self.arduino.in_waiting < 9:
            time.sleep(0)

        data = unpack("<ihB", self.arduino.read(7))
        return data


    def getRawADCReading(self):      
        # request raw ADC value and wait for reply
        self.sendCode("*M", 1234)
        
        # make sure we get a raw adc reading
        command = self.readCommand()
        if command != 'M': return
        reading = self.readInt()

        return reading



        