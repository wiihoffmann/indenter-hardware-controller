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
            # TODO: process the data point

        command = comm.readCommand()
    
    print(point)


def testSequence():
    print("RAW: " + str(comm.getRawADCReading()))

    mp = MeasurementParams()
    performMeasurement(mp)
    
 
    # comm.sendCode("*S", 11111)
    # while comm.arduino.in_waiting < 4:
    #     time.sleep(0)
    # print(comm.arduino.read_all())

    # comm.sendCode("*X", 12345)
    # while comm.arduino.in_waiting < 4:
    #     time.sleep(0)
    # print(comm.arduino.read_all())
    
    # print("RAW: " + str(comm.getRawADCReading()))

    # comm.sendMeasurementBegin(mp)
    # while comm.arduino.in_waiting < 1:
    #     time.sleep(0)
    # print(comm.arduino.read_all())




if __name__ == '__main__':
    while comm.readCommand() != 'R':
        time.sleep(0)
    #comm.arduino.flush()

    for i in range(3):
        print("iteration " + str(i))
        testSequence()


    kill = False
    x = Thread(target=monitor, args=()).start()

    time.sleep(1.5)
    kill = True
    sys.exit()



