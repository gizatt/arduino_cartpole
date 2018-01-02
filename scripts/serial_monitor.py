from __future__ import print_function
import glob
import struct
import time
import numpy as np
import threading
import serial
import sys

# the interface stuff
from PyQt4 import QtCore, QtGui
import pyqtgraph as pg

from data_plot_widget import DataPlotWidget

def look_for_available_ports():
    """
    find available serial ports to Arduino
    """
    available_ports = glob.glob('/dev/ttyACM*')
    print("Available porst: ")
    print(available_ports)

    return available_ports


def get_time_millis():
    return(int(round(time.time() * 1000)))


def get_time_seconds():
    return(int(round(time.time() * 1000000)))


def print_values(values):
    print("------ ONE MORE MEASUREMENT ------")
    print("Accelerations: ")
    print(values[0:2])
    print("Offsets: ")
    print(values[3:6])
    print("Angular rate: ")
    print(values[2])
    print("State: ")
    print(values[6:11])
    print("Zero angle info:")
    print(values[14:16])

class ReadFromArduino(object):
    """A class to read the serial messages from Arduino. The code running on Arduino
    can for example be the ArduinoSide_LSM9DS0 sketch."""

    def __init__(self, port, SIZE_STRUCT=64, verbose=0):
        self.port = port
        self.millis = get_time_millis()
        self.SIZE_STRUCT = SIZE_STRUCT
        self.verbose = verbose
        self.latest_values = -1
        self.t_init = get_time_millis()
        self.t = 0

        self.port.flushInput()

    def read_one_value(self):
        """Wait for next serial message from the Arduino, and read the whole
        message as a structure."""
        read = False

        while not read:
            myByte = self.port.read(1)
            if myByte == 'S':
                data = self.port.read(self.SIZE_STRUCT)
                myByte = self.port.read(1)
                print(data, myByte)
                if myByte == 'E':
                    self.t = (get_time_millis() - self.t_init) / 1000.0

                    # is  a valid message struct
                    new_values = struct.unpack('<ffffffffffffffff', data)

                    current_time = get_time_millis()
                    time_elapsed = current_time - self.millis
                    self.millis = current_time

                    read = True

                    self.latest_values = np.array(new_values)

                    if self.verbose > 1:
                        print("Time elapsed since last (ms): " + str(time_elapsed))
                        print_values(new_values)

                    return(True)

        return(False)
 
if __name__ == "__main__":
    ports = look_for_available_ports()
    usb_port = serial.Serial(ports[0], baudrate=115200, timeout=0.5)
    reader = ReadFromArduino(usb_port, verbose=10)

    app = QtGui.QApplication(sys.argv)
    window = QtGui.QWidget()
    windowLayout = QtGui.QHBoxLayout()
    
    statePlotter = DataPlotWidget(5, title="States", alsoNumeric=True, histLength=1000)
    windowLayout.addWidget(statePlotter)

    window.setLayout(windowLayout)
    window.show()

    keep_going = True
    def readerThread():
        while (keep_going):
            if reader.read_one_value():
                t = time.time()
                statePlotter.addDataPoint(0, t, reader.latest_values[6])
                statePlotter.addDataPoint(1, t, reader.latest_values[7])
                statePlotter.addDataPoint(2, t, reader.latest_values[8])
                statePlotter.addDataPoint(3, t, reader.latest_values[9])
                statePlotter.addDataPoint(4, t, reader.latest_values[10])

    t = threading.Thread(target=readerThread)
    t.start()

    app.exec_()
    keep_going = False
    t.join()
