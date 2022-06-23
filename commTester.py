from threading import Thread
import serial
import time
from struct import *
import sys


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

    # print(len(dataToSend))
    # print(dataToSend)
    print("HOST: send measurement init sequence")

    arduino.write(dataToSend)


def sendCode(preamble, int):
    preamble = bytes(preamble, 'utf-8')
    dataToSend = pack("<%dsh" % (len(preamble)), preamble, int)
    arduino.write(dataToSend)


def monitor():
    while not kill:
        try:
            if arduino.in_waiting:
                print(arduino.readline().decode('utf-8'), end = '') # printing the value
        except Exception as e:
            pass


arduino = serial.Serial(port='/dev/ttyACM0', baudrate=2000000, timeout=None)
arduino.flush()
kill = False
x = Thread(target=monitor, args=()).start()
time.sleep(3)


TIMEOUT = .1


sendMeasurementBegin()

print("HOST: sending more commands")

time.sleep(TIMEOUT)
sendCode("*X", 12345)
time.sleep(TIMEOUT)
sendCode("*X",-12345)
time.sleep(TIMEOUT)
sendCode("*X", 12321)

# time.sleep(TIMEOUT)
# sendMeasurementBegin()
# time.sleep(TIMEOUT)
# sendMeasurementBegin()


time.sleep(.2)
kill = True
sys.exit()
