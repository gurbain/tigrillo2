
import datetime
import json
import time

import rospy as ros
from rospy_message_converter import message_converter
from std_msgs.msg import String
from std_srvs.srv import *

from tigrillo_io.srv import Calibration, CalibrationResponse, Frequency, FrequencyResponse
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
        self.calibration_data = None

    def start(self, calib_file):

        self.load_calib(calib_file)

        ros.logwarn("Sensors properly initialized!")

    def change_uart_period(self, period):

        line = "F"
        line += str(period)
        self.uart.write(line)
        time.sleep(0.02)

        ack = self.uart.read_f_ack()
        if ack["success"]:
            ros.logwarn("UART sensors period properly changed to " + str(period) + " us!")
        else:
            ros.logwarn("Cannot change UART sensors frequency, check the log!")

        return ack

    def reset_sensors(self):

        line = "R"
        self.uart.write(line)
        time.sleep(0.01)

        ack = self.uart.read_r_ack()
        if ack["success"]:
            ros.logwarn("UART Sensors properly calibrated!")
        else:
            ros.logwarn("Cannot reset properly the UART sensors, check the log!")

        return ack

    def load_calib(self, file):

        data = utils.load_calib_file(file)
        self.f_fr = interp1d(data["fr"], data["a_fr"], kind='cubic', fill_value="extrapolate")
        self.f_fl = interp1d(data["fl"], data["a_fl"], kind='cubic', fill_value="extrapolate")
        self.f_br = interp1d(data["br"], data["a_br"], kind='cubic', fill_value="extrapolate")
        self.f_bl = interp1d(data["bl"], data["a_bl"], kind='cubic', fill_value="extrapolate")

        ros.logwarn("UART sensors calibration loaded from file!")

    def save_calib(self, data, file):

        utils.save_calib_file(data, file)
        ros.logwarn("UART sensors calibration saved in file!")

    def get_measure(self):

        measure = self.uart.read_data()
        print measure,  self.calibration_data
        if not measure:
            measure = dict()
            ros.loginfo("Cannot retrieve UART sensor data! Please increase sensor reading period!")

        return measure


class UARTActuators:

    def __init__(self, uart):

        self.uart = uart

        self.mul_factor = 1
        self.offset = 115

    def load_calib(self, file):

        data = utils.load_calib_file(filename)
        self.mul_factor = data["mul_factor"]
        self.offset = data["offset"]

        ros.logwarn("Actuators calibration loaded from file!")

    def save_calib(self, data, file):

        self.mul_factor = data["mul_factor"]
        self.offset = data["offset"]
        data = {"mul_factor": self.mul_factor, "offset": self.offset}

        utils.save_calib_file(data, file)
        ros.logwarn("Actuators calibration saved in file!")

    def start(self, calib_file):

        # self.load_calib(calib_file)
        ros.logwarn("Actuators properly initialized!")

    def update(self, update):

        line = "A"
        line += str(self.mul_factor * int(update["FL"]) + self.offset) + ','
        line += str(self.mul_factor * int(update["FR"]) + self.offset) + ','
        line += str(self.mul_factor * int(update["BL"]) + self.offset) + ','
        line += str(self.mul_factor * int(update["BR"]) + self.offset)
        self.uart.write(line)


class UARTROS():


    def __init__(self, ser_port="/dev/ttyS0", ser_baud=921600, usb_port="/dev/ttyACM0", usb_baud=9600, 
                 calib_uart_sens="calib_uart_sens.json" , calib_uart_act="calib_uart_act.json",
                 pub_freq=5, save_all=True, data_folder=None):

        # ROS parameters
        self.node_name = "uartd"
        self.pub_name = "uart_sensors"
        self.srv_freq_name = "uart_set_sens_freq"
        self.srv_load_sens_cal_name = "uart_load_sens_cal"
        self.srv_save_sens_cal_name = "uart_save_sens_cal"
        self.srv_load_act_cal_name = "uart_load_act_cal"
        self.srv_save_act_cal_name = "uart_save_act_cal"
        self.sub_name = "uart_actuators"
        self.pub_rate = pub_freq
        self.queue_size = utils.ROS_QUEUE_SIZE

        self.ser_baud = ser_baud
        self.ser_port = ser_port
        self.usb_baud = usb_baud
        self.usb_port = usb_port
        self.save_all = save_all
        self.data_folder = data_folder

        self.uart = None
        self.sensors = None
        self.sensors_index = 0
        self.actuators = None
        self.actuators_index = 0
        self.calib_uart_sens = calib_uart_sens
        self.calib_uart_act = calib_uart_act

        self.date_zero_s = None
        self.date_zero_a = None

        if save_all:
            if data_folder is None:
                data_folder = utils.RESULTS_FOLDER
            utils.mkdir(data_folder)
            self.file_s = data_folder + "/uart_sensors_" + utils.timestamp() + ".csv"
            self.file_a = data_folder + "/uart_actuators_" + utils.timestamp() + ".csv"

    def start(self):

        # Setup and start uart daemon
        self.uart = uart_daemon.UARTDaemon(self.ser_port, self.ser_baud, self.usb_port, self.usb_baud)
        self.uart.start()

        self.sensors = UARTSensors(self.uart)
        self.actuators = UARTActuators(self.uart)
        self.sensors.start(self.calib_uart_sens)
        self.actuators.start(self.calib_uart_act)

        self.start_ros_node()

        return

    def get_last_sensors(self):

        measure = self.sensors.get_measure()
        nows = datetime.datetime.now()
        if self.date_zero_s is None:
            self.date_zero_s = nows.minute*60 + nows.second + nows.microsecond/1000000.0
        measure["Run Time"] = str(nows.minute*60 + nows.second + nows.microsecond/1000000.0 - self.date_zero_s)

        if self.save_all:
            utils.save_csv_row(measure, self.file_s, self.sensors_index)

        self.sensors_index += 1
        return measure

    def update_actuators(self, update):

        self.actuators.update(update)
        nowa = datetime.datetime.now()
        if self.date_zero_a is None:
            self.date_zero_a = nowa.minute*60 + nowa.second + nowa.microsecond/1000000.0
        update["Run Time"] = str(nowa.minute*60 + nowa.second + nowa.microsecond/1000000.0 - self.date_zero_a)

        if self.save_all:
            utils.save_csv_row(update, self.file_a, self.actuators_index)

        self.actuators_index += 1
        return

    def set_sensor_period(self, period=0.01):

        return self.sensors.changeUARTPeriod(int(period * 1000000))

    def reset_sensors_calib(self):

        return self.sensors.resetCalib()

    def __ros_sub(self, msg):

        self.update_actuators(utils.dict_keys_to_str(json.loads(msg.data)))

    def __ros_pub(self):

        rate = ros.Rate(self.pub_rate)

        while not ros.is_shutdown():
            rate = ros.Rate(self.pub_rate)
            measure = self.get_last_sensors()
            self.pub.publish(json.dumps(measure))
            rate.sleep()

        return

    def __ros_freq_srv(self, msg):

        if msg.freq > 0:
            ack = self.set_sensor_period(0.99/float(msg.freq))
            if ack["success"]:
                self.pub_rate = msg.freq
            ros.logdebug("UART Frequency changed with ack message:" + str(ack["msg"]))
        else:
            ack = {"success": False, "msg": "Frequency format not supported: " + str(msg.freq)}
        return FrequencyResponse(ack["success"], ack["msg"])

    def __ros_save_sens_cal_srv(self, msg):

        self.sensors.save_calib(msg.data, msg.filename)

        return CalibrationResponse(True, "")

    def __ros_load_sens_cal_srv(self, msg):

        self.sensors.load_calib(msg.filename)

        return CalibrationResponse(True, "")

    def __ros_save_act_cal_srv(self, msg):

        self.actuators.save_calib(msg.data, msg.filename)

        return CalibrationResponse(True, "")

    def __ros_load_act_cal_srv(self, msg):

        self.actuators.load_calib(msg.filename)

        return CalibrationResponse(True, "")

    def start_ros_node(self):

        ros.init_node(self.node_name, log_level=ros.INFO)
        self.pub = ros.Publisher(self.pub_name, String, queue_size=self.queue_size)
        self.sub = ros.Subscriber(self.sub_name, String, callback=self.__ros_sub, queue_size=self.queue_size)
        
        self.srv_save_sens_cal = ros.Service(self.srv_save_sens_cal_name, Calibration, self.__ros_save_sens_cal_srv)
        self.srv_load_sens_cal = ros.Service(self.srv_load_sens_cal_name, Calibration,  self.__ros_load_sens_cal_srv)
        self.srv_save_act_cal = ros.Service(self.srv_save_act_cal_name, Calibration, self.__ros_save_act_cal_srv)
        self.srv_load_act_cal = ros.Service(self.srv_load_act_cal_name, Calibration, self.__ros_load_act_cal_srv)

        self.srv_freq = ros.Service(self.srv_freq_name, Frequency, self.__ros_freq_srv)

        try:
            self.__ros_pub()
        except ros.ROSInterruptException:
            pass

        return


if __name__ == "__main__":

    uartros = UARTROS(ser_port="/dev/ttyS0", ser_baud=921600, usb_port="/dev/ttyACM0", usb_baud=9600, pub_freq=10)
    uartros.start()
