from threading import Thread
import time
from struct import *
import sys
from tracemalloc import start
from Communicator import *

comm = Communicator()


def monitor():
    while not kill:
        try:
            if comm.arduino.in_waiting:
                print(comm.arduino.read_all().decode('utf-8'), end = '') # printing the value
                # print(comm.arduino.readline().hex()) # printing the value
        except Exception as e:
            pass


def performMeasurement(params):
    comm.sendMeasurementBegin(params)
    # wait for a reply
    command = comm.readCommand()
    
    point = None

    # process data until we see the complete flag
    while command != 'C':
        if command != 'D':
            print("Error: exptected command D but got " + command)
            print("point before the error: " + str(point))
        else:
            point = comm.readDataPoint()
            print(point)
            # TODO: process the data point

        command = comm.readCommand()
    
    print(point)
    print(point[0]/point[1]*1000)


def testSequence():
    print("RAW: " + str(comm.getRawADCReading()))

    mp = MeasurementParams()
    performMeasurement(mp)
    
 
    comm.sendCode("*S", 22222)

    comm.sendCode("*X", 12345)

    comm.sendCode("*Y", -12345)

    comm.sendCode("*Z", 12321)

    # print("RAW: " + str(comm.getRawADCReading()))

    time.sleep(5)


def testSequence2():
    startTime = time.time()
    while(time.time() - startTime < 10):
        # comm.sendCode("*S", 200)

        comm.sendCode("*X", 300)

        comm.sendCode("*Y", -400)

        comm.sendCode("*Z", 500)

        time.sleep(.001)



if __name__ == '__main__':
    while comm.readCommand() != 'R':
        time.sleep(0)
    

    # for i in range(1):
    #     print("iteration " + str(i))
    #     testSequence()
    testSequence2()

    kill = False
    x = Thread(target=monitor, args=()).start()
    time.sleep(1.5)
    kill = True

    sys.exit()



