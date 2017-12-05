
from tigrillo_io.srv import Frequency, FrequencyResponse
import tigrillo_io
from tigrillo_io import BNO055, utils

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

    def __init__(self, i2c_rst_pin, i2c_calib_file):

       self.i2c_rst_pin = i2c_rst_pin
       self.i2c_calib_file = i2c_calib_file

   
    def start(self):

        self.imu = BNO055.BNO055(rst=int(self.i2c_rst_pin))

        if not self.imu.begin():
           raise RuntimeError('Failed to initialize IMU! Please, check the connection')
        else:
           ros.logwarn("I2C Sensors properly initialized!")
           self.printStatus()

        return

    def printStatus(self):

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


    def resetCalib(self):

        with open(self.i2c_calib_file, 'r') as cal_file:
            data = json.load(cal_file)
            self.imu.set_calibration(data)

        ros.logwarn("Sensors properly calibrated!")

    def getMeasure(self):

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

    def __init__(self, i2c_rst_pin="18", i2c_calib_file=utils.I2C_CALIB_FILE, pub_freq="5", save_all=True, data_folder=None):

        # ROS parameters
        self.node_name = "i2c"
        self.pub_name = "i2c_sensors"
        self.srv_freq_name = "i2c_set_sens_freq"
        self.srv_rst_name = "i2c_rst_sens_zero"
        self.pub_rate = pub_freq
        self.queue_size = utils.ROS_QUEUE_SIZE

        self.save_all = save_all
        self.data_folder = data_folder
        self.sensors = None
        self.sensors_index = 0

        self.i2c_rst_pin = i2c_rst_pin
        self.i2c_calib_file = i2c_calib_file

        self. runtime = 0

        if save_all:
            if data_folder is None:
                data_folder = utils.RESULTS_FOLDER
            utils.mkdir(data_folder)
            self.file_s = data_folder + "/i2c_sensors_" + utils.timestamp() + ".csv"


    def start(self):

        self.sensors = I2CSensors(self.i2c_rst_pin, self.i2c_calib_file)
        self.sensors.start()

        self.start_ros_node()

        return

    def get_last_sensors(self):

        measure = self.sensors.getMeasure()

        if self.save_all:
            measure["Run Time"] = self.runtime
            utils.save_csv_row(measure, self.file_s, self.sensors_index)

        self.sensors_index += 1
        return measure

    def reset_sensors_calib(self):

        return self.sensors.resetCalib()

    def __ros_pub(self):

        while not ros.is_shutdown():
            rate = ros.Rate(self.pub_rate)
            measure = self.get_last_sensors()
            ros.logdebug("Last I2C sensor measure: " + str(measure))
            self.pub.publish(json.dumps(measure))
            self.runtime += 1.0/self.pub_rate
            rate.sleep()

        return

    def __ros_freq_srv(self, msg):

        self.pub_rate = msg.freq
        ros.logwarn("I2C sensors frequency properly changed to " + str(msg.freq) + " Hz!")
        return FrequencyResponse(True, "Success!")

    def __ros_rst_srv(self, msg):

        self.reset_sensors_calib()
        return TriggerResponse(True, "Success!")

    def start_ros_node(self):

        ros.init_node(self.node_name, log_level=ros.INFO)
        self.pub = ros.Publisher(self.pub_name, String, queue_size=self.queue_size)
        self.srv_rst = ros.Service(self.srv_rst_name, Trigger, self.__ros_rst_srv)
        self.srv_freq = ros.Service(self.srv_freq_name, Frequency, self.__ros_freq_srv)

        try:
            self.__ros_pub()
        except ros.ROSInterruptException:
            pass

        return


if __name__ == "__main__":

    i2cros = I2CROS(i2c_rst_pin="18", i2c_calib_file="calibration.json", pub_freq=10)
    i2cros.start()
