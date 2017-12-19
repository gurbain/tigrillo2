
from tigrillo_io.srv import Calibration, CalibrationResponse, Frequency, FrequencyResponse
import tigrillo_io
from tigrillo_io import BNO055, utils

import datetime
import json
import time

import rospy as ros
from rospy_message_converter import message_converter
from std_msgs.msg import String
from std_srvs.srv import *


__author__ = "Gabriel Urbain" 
__copyright__ = "Copyright 2017, Human Brain Projet, SP10"

__license__ = "MIT" 
__version__ = "2.0" 
__maintainer__ = "Gabriel Urbain"
__email__ = "gabriel.urbain@ugent.be" 
__status__ = "Research" 
__date__ = "September 11th, 2017"



class I2CSensors:

    def __init__(self, i2c_rst_pin):

       self.i2c_rst_pin = i2c_rst_pin

   
    def start(self, calib_file):

        self.imu = BNO055.BNO055(rst=int(self.i2c_rst_pin))

        if not self.imu.begin():
           raise RuntimeError('Failed to initialize IMU! Please, check the connection')
        else:
           ros.logwarn("I2C Sensors properly initialized!")
           self.print_status()

        self.load_calib(calib_file)

        return

    def print_status(self):

        status, self_test, error = self.imu.get_system_status()
        sw, bl, acc, mag, gyro = self.imu.get_revision()
        cal_sys, cal_gyro, cal_acc, cal_mag = self.imu.get_calibration_status()

        ros.logwarn('IMU STATUS')
        ros.logwarn('\tSystem status: {0}'.format(status))
        ros.logwarn('\tSelf test result (0x0F is normal): 0x{0:02X}'.format(self_test))
        if status == 0x01:
            ros.logwarn('\tSystem error: {0}'.format(error))
            ros.logwarn('\tSee datasheet section 4.3.59 for the meaning.')
        ros.logwarn('\tSoftware version:   {0}'.format(sw))
        ros.logwarn('\tBootloader version: {0}'.format(bl))
        ros.logwarn('\tAccelerometer ID:   0x{0:02X}'.format(acc))
        ros.logwarn('\tMagnetometer ID:    0x{0:02X}'.format(mag))
        ros.logwarn('\tGyroscope ID:       0x{0:02X}\n'.format(gyro))
        ros.logwarn('\tCalibration System status:          {0}'.format(cal_sys))
        ros.logwarn('\tCalibration Accelerometers status:  0x{0:02X}'.format(cal_acc))
        ros.logwarn('\tCalibration Magnetometer status:    0x{0:02X}'.format(cal_mag))
        ros.logwarn('\tCalibration Gyroscope status:       0x{0:02X}\n'.format(cal_gyro))

    def load_calib(self, file):

        data = utils.load_calib_file(file)
        self.imu.set_calibration(data)

        ros.logwarn("IMU sensors calibration loaded from " + str(file) + "!")

    def save_calib(self, data, file):

        calib = self.imu.get_calibration()
        calib_str = json.dumps(calib)
        utils.save_calib_file(calib_str, file)

        ros.logwarn("IMU sensors calibration saved in " + str(file) + "!")
        return calib_str

    def get_measure(self):

        imu_timestamp = time.time()
        h, r, p = self.imu.read_euler()
        a_x, a_y, a_z = self.imu.read_linear_acceleration()
        g_x, g_y, g_z = self.imu.read_gravity()
        calib_sys, calib_gyro, calib_a, calib_mag = self.imu.get_calibration_status()
        imu_read_time = time.time() - imu_timestamp

        measure = {"IMU Time Stamp": imu_timestamp, "IMU IO Time": imu_read_time,
                   "Heading": h, "Roll": r, "Pitch": p, "IMU Calib System": calib_sys,
                   "IMU Calib Gyroscope": calib_gyro, "IMU Calib Accelerometer": calib_a,
                   "IMU Calib Magnetometer": calib_mag, "Acceleration X": a_x,
                   "Acceleration Y": a_y, "Acceleration Z": a_z, "Gravity X": g_x,
                   "Gravity Y": g_y, "Gravity Z": g_z}

        return measure


class I2CROS():

    def __init__(self, i2c_rst_pin="18", pub_freq="5", calib_i2c="calib_i2c.json", 
                 save_all=True, data_folder=None):

        # ROS parameters
        self.node_name = "i2c"
        self.pub_name = "i2c_sensors"
        self.srv_freq_name = "i2c_set_sens_freq"
        self.srv_save_cal_name = "i2c_save_cal"
        self.srv_load_cal_name = "i2c_load_cal"
        self.pub_rate = pub_freq
        self.queue_size = utils.ROS_QUEUE_SIZE

        self.save_all = save_all
        self.data_folder = data_folder
        self.sensors = None
        self.sensors_index = 0
        self.calib_i2c = calib_i2c

        self.i2c_rst_pin = i2c_rst_pin

        self.date_zero_s = None
        self.date_zero_a = None

        if save_all:
            if data_folder is None:
                data_folder = utils.RESULTS_FOLDER
            utils.mkdir(data_folder)
            self.file_s = data_folder + "/i2c_sensors_" + utils.timestamp() + ".csv"


    def start(self):

        self.sensors = I2CSensors(self.i2c_rst_pin)
        self.sensors.start(self.calib_i2c)

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

    def __ros_pub(self):

        while not ros.is_shutdown():
            rate = ros.Rate(self.pub_rate)
            measure = self.get_last_sensors()
            self.pub.publish(json.dumps(measure))
            rate.sleep()

        return

    def __ros_freq_srv(self, msg):

        if msg.freq > 0:
            self.pub_rate = msg.freq
            ros.logwarn("I2C sensors frequency properly changed to " + str(msg.freq) + " Hz!")
            ack = {"success":True, "msg": "Success!"}
        else:
            ack = {"success": False, "msg": "Frequency format not supported: " + str(msg.freq)}

        return FrequencyResponse(ack["success"], ack["msg"])

    def __ros_save_cal_srv(self, msg):

        calib = self.sensors.save_calib(msg.data, msg.filename)

        return CalibrationResponse(True, calib)

    def __ros_load_cal_srv(self, msg):

        self.sensors.load_calib(msg.filename)

        return CalibrationResponse(True, "")

    def start_ros_node(self):

        ros.init_node(self.node_name, log_level=ros.INFO)
        self.pub = ros.Publisher(self.pub_name, String, queue_size=self.queue_size)
        self.srv_save_cal = ros.Service(self.srv_save_cal_name, Calibration, self.__ros_save_cal_srv)
        self.srv_load_cal = ros.Service(self.srv_load_cal_name, Calibration, self.__ros_load_cal_srv)
        self.srv_freq = ros.Service(self.srv_freq_name, Frequency, self.__ros_freq_srv)

        try:
            self.__ros_pub()
        except ros.ROSInterruptException:
            pass

        return


if __name__ == "__main__":

    i2cros = I2CROS(i2c_rst_pin="18", pub_freq=10)
    i2cros.start()
