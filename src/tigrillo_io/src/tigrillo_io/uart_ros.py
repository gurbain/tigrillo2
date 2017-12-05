import json
import time

import rospy as ros
from rospy_message_converter import message_converter
from std_msgs.msg import String
from std_srvs.srv import *

from tigrillo_io.srv import Frequency, FrequencyResponse
import tigrillo_io
from tigrillo_io import BNO055, uart_daemon, utils


__author__ = "Gabriel Urbain" 
__copyright__ = "Copyright 2017, Human Brain Projet, SP10"

__license__ = "MIT" 
__version__ = "2.0" 
__maintainer__ = "Gabriel Urbain"
__email__ = "gabriel.urbain@ugent.be" 
__status__ = "Research" 
__date__ = "September 11th, 2017"



class UARTSensors:

    def __init__(self, uart):

        self.uart = uart

    def start(self):

        ros.logwarn("Sensors properly initialized!")

    def changeUARTPeriod(self, period):

        line = "F"
        line += str(period)
        line += "\n\r"
        self.uart.write(line)
        time.sleep(0.02)

        ack = self.uart.read_f_ack()
        if ack["success"]:
            ros.logwarn("UART sensors period properly changed to " + str(period) + " us!")
        else:
            ros.logwarn("Cannot change UART sensors frequency, check the log!")

        return ack

    def resetCalib(self):

        line = "R\n\r"
        self.uart.write(line)
        time.sleep(0.01)

        ack = self.uart.read_r_ack()
        if ack["success"]:
            ros.logwarn("UART Sensors properly calibrated!")
        else:
            ros.logwarn("Cannot reset properly the UART sensors, check the log!")

        return ack

    def getMeasure(self):

        measure = self.uart.read_data()
        if not measure:
            measure = dict()
            ros.loginfo("Cannot retrieve UART sensor data! Please increase sensor reading period!")

        return measure


class UARTActuators:

    def __init__(self, uart):

        self.uart = uart

    def start(self):

        ros.logwarn("Actuators properly initialized!")

    def update(self, update):

        line = "A"
        line += str(int(update["FL"])) + ','
        line += str(int(update["FR"])) + ','
        line += str(int(update["BL"])) + ','
        line += str(int(update["BR"])) + '\r\n'
        self.uart.write(line)


class UARTROS():

    def __init__(self, uart_port="/dev/ttyAMA0", uart_baud=921600, pub_freq=5, save_all=True, data_folder=None):

        # ROS parameters
        self.node_name = "uartd"
        self.pub_name = "uart_sensors"
        self.srv_freq_name = "uart_set_sens_freq"
        self.srv_rst_name = "uart_rst_sens_zero"
        self.sub_name = "uart_actuators"
        self.pub_rate = pub_freq
        self.queue_size = utils.ROS_QUEUE_SIZE

        self.uart_baud = uart_baud
        self.uart_port = uart_port
        self.save_all = save_all
        self.data_folder = data_folder

        self.uart = None
        self.sensors = None
        self.sensors_index = 0
        self.actuators = None
        self.actuators_index = 0

        self. runtime = 0

        if save_all:
            if data_folder is None:
                data_folder = utils.RESULTS_FOLDER
            utils.mkdir(data_folder)
            self.file_s = data_folder + "/uart_sensors_" + utils.timestamp() + ".csv"
            self.file_a = data_folder + "/uart_actuators_" + utils.timestamp() + ".csv"

    def start(self):

        # Setup and start uart daemon
        self.uart = uart_daemon.UARTDaemon(self.uart_port, self.uart_baud)
        self.uart.start()

        self.sensors = UARTSensors(self.uart)
        self.actuators = UARTActuators(self.uart)
        self.sensors.start()
        self.actuators.start()

        self.start_ros_node()

        return

    def get_last_sensors(self):

        measure = self.sensors.getMeasure()

        if self.save_all:
            measure["Run Time"] = self.runtime
            utils.save_csv_row(measure, self.file_s, self.sensors_index)

        self.sensors_index += 1
        return measure

    def update_actuators(self, update, t_run):

        self.actuators.update(update)

        if self.save_all:
            update["Run Time"] = t_run
            utils.save_csv_row(update, self.file_a, self.actuators_index)

        self.actuators_index += 1
        return

    def set_sensor_period(self, period=0.01):

        return self.sensors.changeUARTPeriod(int(period * 1000000))

    def reset_sensors_calib(self):

        return self.sensors.resetCalib()

    def __ros_sub(self, msg):

        return 

    def __ros_pub(self):

        while not ros.is_shutdown():
            rate = ros.Rate(self.pub_rate)
            measure = self.get_last_sensors()
            ros.logdebug("Last UART sensor measure: " + str(measure))
            self.pub.publish(json.dumps(measure))
            self.runtime += 1.0/self.pub_rate
            rate.sleep()

        return

    def __ros_freq_srv(self, msg):

        ack = self.set_sensor_period(0.99/float(msg.freq))
        if ack["success"]:
            self.pub_rate = msg.freq
        return FrequencyResponse(ack["success"], ack["msg"])

    def __ros_rst_srv(self, msg):

        ack = self.reset_sensors_calib()
        return TriggerResponse(ack["success"], ack["msg"])

    def start_ros_node(self):

        ros.init_node(self.node_name, log_level=ros.INFO)
        self.pub = ros.Publisher(self.pub_name, String, queue_size=self.queue_size)
        self.sub = ros.Subscriber(self.sub_name, String, callback=self.__ros_sub, queue_size=self.queue_size)
        self.srv_rst = ros.Service(self.srv_rst_name, Trigger, self.__ros_rst_srv)
        self.srv_freq = ros.Service(self.srv_freq_name, Frequency, self.__ros_freq_srv)

        try:
            self.__ros_pub()
        except ros.ROSInterruptException:
            pass

        return


if __name__ == "__main__":

    uartros = UARTROS(uart_port="/dev/ttyS0", uart_baud=921600, pub_freq=10)
    uartros.start()
