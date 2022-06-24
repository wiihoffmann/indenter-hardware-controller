from threading import Thread
import time
from struct import *
import sys
from Communicator import *

comm = Communicator()

TIMEOUT = .5


def monitor():
    while not kill:
        try:
            if comm.arduino.in_waiting:
                print(comm.arduino.read_all().decode('utf-8'), end = '') # printing the value
        except Exception as e:
            pass


if __name__ == '__main__':
    time.sleep(3)


    comm.calibrate()


    # mp = MeasurementParams()
    # comm.sendMeasurementBegin(mp)

    # time.sleep(4)
    # comm.sendCode("*S", 12345)

    # time.sleep(TIMEOUT)
    # comm.sendCode("*X", 12345)

    # time.sleep(TIMEOUT)
    # comm.sendCode("*X",-12345)

    # time.sleep(TIMEOUT)
    # comm.sendCode("*X", 12321)
    
    # # time.sleep(TIMEOUT)
    # # sendMeasurementBegin()

    # # time.sleep(TIMEOUT)
    # # sendMeasurementBegin()

    kill = False
    x = Thread(target=monitor, args=()).start()

    time.sleep(1.2)
    kill = True
    sys.exit()



