"""
This file contains all other methods
"""

import ast
from collections import deque
import copy
import rospy as ros
import serial
import threading
import time


__author__ = "Gabriel Urbain" 
__copyright__ = "Copyright 2017, Human Brain Projet, SP10"

__license__ = "MIT" 
__version__ = "2.0" 
__maintainer__ = "Gabriel Urbain"
__email__ = "gabriel.urbain@ugent.be" 
__status__ = "Research" 
__date__ = "September 11th, 2017"


MAX_BUFFER_SIZE = 100


class UARTDaemon(threading.Thread):
    """
    This UART daemon monitors the use of a serial IO channel and store all receeeived
    data in a buffer for asynchronous usage
    """

    def __init__(self, ser_port, ser_baud, usb_port, usb_baud):

        super(UARTDaemon, self).__init__()

        self.usb_port = usb_port
        self.usb_baud = usb_baud
        self.usb_conn = None

        self.ser_port = ser_port
        self.ser_baud = ser_baud
        self.ser_conn = None

        self.data_buffer = None
        self.ack_buffer = deque([])

        self.stop = True
        self.read_period = 0.0001
        self.daemon = True

    def start(self):
        """ Initialize and configure serial port """

        self.ser_conn = serial.Serial(self.ser_port, self.ser_baud)
        self.usb_conn = serial.Serial(self.usb_port, self.usb_baud)
        if not self.ser_conn.isOpen():
            self.ser_conn.open()

        if not self.usb_conn.isOpen():
            self.usb_conn.open()

        self.stop = False
        super(UARTDaemon, self).start()

    def stop(self):
        """ Raise a stop flag to stop the daemon """

        self.stop = True
        self.ser_conn.close()
        self.usb_conn.close()

    def _add_data(self, data):
        """ Add a data line in the uart buffer """

        if "Previous Time Stamp" and "Time Stamp" and "End of Reading Time Stamp" in data:
            ts = float(data["Time Stamp"]) / 1000000
            eor = float(data["End of Reading Time Stamp"]) / 1000000
            pts = float(data["Previous Time Stamp"]) / 1000000
            data["UART IO Time"] = eor - ts
            data["UART Time Stamp"] = ts
            data["UART Loop Time"] = ts - pts
            del data["End of Reading Time Stamp"]
            del data["Previous Time Stamp"]
            del data["Time Stamp"]
        if "Sensors values" in data:
            sv = data["Sensors values"]
            data.update(sv)
            del data["Sensors values"]

        self.data_buffer = data

    def _add_ack(self, ack):
        """ Add a ack line in the uart buffer """

        if ack["Instruction"] == 'A':
            if not "success" in ack["Data"].lower():
                ros.logerr("Arg, data not received! Ack: " + str(ack))

        # TODO: PRINT IF ACK FAILED
        self.ack_buffer.append(ack)
        if len(self.ack_buffer) > MAX_BUFFER_SIZE:
            self.ack_buffer.popleft()

    def read_data(self):
        """ Read the last data line in the uart buffer """

        return self.data_buffer

    def read_r_ack(self):
        """ Read the last rst ack line in the uart buffer """

        if self.ack_buffer:
            for a in self.ack_buffer:
                if a["Instruction"] == 'R':
                    ack = {"success": True, "msg": a["Data"]}
                    if not "success" in a["Data"].lower():
                        ack["success"] = False
                    self.ack_buffer.remove(a)
                    return ack
        else:
            return {"success": False, "msg": "No ACK reveived!"}

    def read_f_ack(self):
        """ Read the last frequency ack line in the uart buffer """

        if self.ack_buffer:
            for a in self.ack_buffer:
                if a["Instruction"] == 'F':
                    ack = {"success": True, "msg": a["Data"]}
                    if not "success" in a["Data"].lower():
                        ack["success"] = False
                    self.ack_buffer.remove(a)
                    return ack
        else:
            return {"success": False, "msg": "No ACK reveived!"}

    def write(self, line):
        """ Write a line in the uart channel"""

        self.usb_conn.write(line)

    def run(self):
        """ Read and populate the buffer"""

        word = ""
        vals = []
        while not self.stop:
            for read in self.ser_conn.read():
                ch = str(read)
                if ch != "," and ch !=";":
                    word += ch
                if ch == ",":
                    try:
                        vals.append(int(word))
                    except ValueError:
                        pass
                    word = ""

                if ch == ";":
                    try:
                        vals.append(int(word))
                    except ValueError:
                        pass
                    word = ""
                    try:
                        if len(vals) == 4:
                            self._add_data({"Front Right": vals[0], "Front Left": vals[1],
                                            "Back Right": vals[2], "Back Left": vals[3]})
                    except ValueError:
                        pass
                    vals = []


if __name__ == "__main__":

    uart = UARTDaemon(ser_port="/dev/ttyAMA0", ser_baud=921600, usb_port="/dev/ttyACM0", usb_baud=9600)
    uart.start()
    uart.write("F50000")
    time.sleep(2)
    print uart.read_f_ack()
