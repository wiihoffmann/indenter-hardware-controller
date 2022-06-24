from dataclasses import dataclass
import time
import serial
from struct import *



@dataclass
class MeasurementParams:
    preamble = bytes("*B", 'utf-8') #     start measurement
    calFactor = 1                   #     double calFactor;
    preload = 2                     #     uint8_t preload;
    preloadTime = 3                 #     uint8_t preloadTime;
    maxLoad = 4                     #     uint8_t maxLoad;
    maxLoadTime = 5                 #     uint8_t maxLoadTime;
    stepDelay = 700                 #     uint16_t stepDelay;
    holdDownDelay = 7               #     uint16_t holdDownDelay;
    holdUpDelay = 8                 #     uint16_t holdUpDelay;



class Communicator:

    def __init__(self):
        self.arduino = serial.Serial(port='/dev/ttyACM1', baudrate=2000000, timeout=None)
        self.arduino.flush()


    def sendMeasurementBegin(self, params):
        dataToSend = pack("<%dsfBBBBHHH" % (len(params.preamble)), params.preamble, params.calFactor, params.preload, params.preloadTime, params.maxLoad, params.maxLoadTime, params.stepDelay, params.holdDownDelay, params.holdUpDelay)
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
        
        if self.arduino.read_until(b'*', self.arduino.in_waiting):
            return self.arduino.read().decode('utf-8')
        
        print("trying again")
        self.readCommand()


    def readInt(self):
        data = self.arduino.read(2)
        value = int.from_bytes(data, byteorder='little', signed=True)
        return value


    def readDataPoint(self):
        # wait for the data point to arrive
        while self.arduino.in_waiting < 9:
            time.sleep(0)

        data = unpack("<ifB", self.arduino.read(9))
        return data

    def getRawADCReading(self):      
        # request raw ADC value and wait for reply
        self.sendCode("*M", 1234)
        
        # make sure we get a raw adc reading
        command = self.readCommand()
        if command != 'M': return
        reading = self.readInt()

        return reading



        