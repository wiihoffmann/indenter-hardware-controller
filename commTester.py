import serial
import time
from struct import *


def sendMeasurementBegin():
    preamble = bytes("*B", 'utf-8') #     start measurement
    calFactor = 1                   #     double calFactor;
    preload = 2                     #     uint8_t preload;
    preloadTime = 3                 #     uint8_t preloadTime;
    maxLoad = 4                     #     uint8_t maxLoad;
    maxLoadTime = 5                 #     uint8_t maxLoadTime;
    stepDelay = 6                   #     uint16_t stepDelay;
    holdDownDelay = 7               #     uint16_t holdDownDelay;
    holdUpDelay = 8                 #     uint16_t holdUpDelay;

    dataToSend = pack("<%dsfBBBBHHH" % (len(preamble)), preamble, calFactor, preload, preloadTime, maxLoad, maxLoadTime, stepDelay, holdDownDelay, holdUpDelay)

    print(len(dataToSend))
    print(dataToSend)

    arduino.write(dataToSend)


def sendCode(preamble, int):
    dataToSend = pack("<%dsfBBBBHHH" % (len(preamble)), preamble, int)
    print(len(dataToSend))
    print(dataToSend)
    arduino.write(dataToSend)



arduino = serial.Serial(port='/dev/ttyACM0', baudrate=2000000)
time.sleep(2)
sendMeasurementBegin()

while True:
    print(arduino.readline().decode('ascii')) # printing the value

