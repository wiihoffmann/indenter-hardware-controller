from threading import Thread
import time
from struct import *
import sys
from Communicator import *

comm = Communicator()


def monitor():
    while not kill:
        try:
            if comm.arduino.in_waiting:
                print(comm.arduino.read_all().decode('utf-8'), end = '') # printing the value
        except Exception as e:
            pass


def testSequence():
    print(comm.getRawADCReading())

    mp = MeasurementParams()
    comm.sendMeasurementBegin(mp)
    while comm.arduino.in_waiting < 1:
        time.sleep(0)
    print(comm.arduino.read_all())

    comm.sendCode("*S", 11111)
    while comm.arduino.in_waiting < 4:
        time.sleep(0)
    print(comm.arduino.read_all())

    comm.sendCode("*X", 12345)
    while comm.arduino.in_waiting < 4:
        time.sleep(0)
    print(comm.arduino.read_all())

    comm.sendCode("*X",-12345)
    while comm.arduino.in_waiting < 4:
        time.sleep(0)
    print(comm.arduino.read_all())

    comm.sendCode("*X", 12321)
    while comm.arduino.in_waiting < 4:
        time.sleep(0)
    print(comm.arduino.read_all())
    
    print(comm.getRawADCReading())

    comm.sendMeasurementBegin(mp)
    while comm.arduino.in_waiting < 1:
        time.sleep(0)
    print(comm.arduino.read_all())

    comm.sendMeasurementBegin(mp)
    while comm.arduino.in_waiting < 1:
        time.sleep(0)
    print(comm.arduino.read_all())



if __name__ == '__main__':
    while comm.readCommand() != b'R':
        time.sleep(0)
    #comm.arduino.flush()

    for i in range(10):
        print("iteration " + str(i))
        testSequence()


    kill = False
    x = Thread(target=monitor, args=()).start()

    time.sleep(1.5)
    kill = True
    sys.exit()



