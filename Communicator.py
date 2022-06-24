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
        self.arduino = serial.Serial(port='/dev/ttyACM0', baudrate=2000000, timeout=None)
        self.arduino.flush()

    def sendMeasurementBegin(self, params):
        print("HOST: send measurement init sequence")

        dataToSend = pack("<%dsfBBBBHHH" % (len(params.preamble)), params.preamble, params.calFactor, params.preload, params.preloadTime, params.maxLoad, params.maxLoadTime, params.stepDelay, params.holdDownDelay, params.holdUpDelay)
        self.arduino.write(dataToSend)


    def sendCode(self, preamble, int):
        preamble = bytes(preamble, 'utf-8')
        dataToSend = pack("<%dsh" % (len(preamble)), preamble, int)
        self.arduino.write(dataToSend)


    def readCommand(self):
        if self.arduino.in_waiting >= 2 and self.arduino.read_until('*', self.arduino.in_waiting):
            data = self.arduino.read(1)
            return data

        return 'L'


    def readInt(self):
        data = self.arduino.read(2)
        print(data)
        value = int.from_bytes(data, byteorder='little', signed=True)
        return value


    def calibrate(self):
        print("apply load and press a button to continue")
        input()
        self.sendCode("*Donkey", 1234)
        while self.arduino.in_waiting < 4:
            time.sleep(100)

        command = self.readCommand()
        print("the command is " + str(command))
        
        print("HOST: reading int")
        point1 = self.readInt()
        print("point 1: " + str(point1))

        # print("apply larger load and press a button to continue")
        # input()
        # self.sendCode("*M", 4321)      
        # while self.arduino.in_waiting < 4:
        #     time.sleep(0)
        # command = self.readCommand()
        # print("the command is " + str(command))
        # point2 = self.readInt()

        # print("point 2: " + str(point2))

        print("Done calibration!")