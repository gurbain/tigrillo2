import json
import math
import rospy as ros
from std_msgs.msg import String
from std_srvs.srv import Trigger
import sys
import thread
import time

from tigrillo_ctrl.srv import Calibration, CalibrationResponse, Frequency, FrequencyResponse
from tigrillo_ctrl.msg import Sensors, Motors, Imu
from tigrillo_ctrl import utils, timing


__author__ = "Gabriel Urbain" 
__copyright__ = "Copyright 2017, Human Brain Projet, SP10"

__license__ = "MIT" 
__version__ = "2.0" 
__maintainer__ = "Gabriel Urbain"
__email__ = "gabriel.urbain@ugent.be" 
__status__ = "Research" 
__date__ = "November 25th, 2017"


class CalibROS():

    def __init__(self, uart_sens_filename="calib_uart_sens.json", uart_act_filename="calib_uart_act.json",
                 i2c_sens_filename="calib_i2c.json"):
        
        self.node_name = "tigrillo_calibration"
        self.i2c_sub_name = "tigrillo_rob/i2c_sensors"
        self.uart_sub_name =  "tigrillo_rob/uart_sensors"
        self.uart_pub_name =  "tigrillo_rob/uart_actuators"

        self.i2c_srv_cal_name = "i2c_save_cal"
        self.i2c_srv_cal = None
        self.uart_srv_sens_cal_name = "uart_save_sens_cal"
        self.uart_srv_sens_cal = None
        self.uart_srv_act_cal_save_name = "uart_save_act_cal"
        self.uart_srv_act_cal_load_name = "uart_load_act_cal"
        self.uart_srv_act_cal_save = None
        self.uart_srv_act_cal_load = None
        
        self.uart_sens_filename = uart_sens_filename
        self.uart_act_filename = uart_act_filename
        self.i2c_sens_filename = i2c_sens_filename

        self.i2c_msg = None
        self.uart_msg = None

        self.k_list = []
        self.queue_size = utils.ROS_QUEUE_SIZE

    def calibrate_imu(self):


        # Wait for the i2c node to be started, then subscribe to its topics
        print("\n === IMU Calibration ===\n")
        ros.logwarn("Waiting for the /tigrillo_io/i2cd node to be started...")
        self.i2c_sub = ros.Subscriber(self.i2c_sub_name, Imu, callback=self.__i2c_ros_sub, queue_size=self.queue_size)
        ros.wait_for_service(self.i2c_srv_cal_name)
        self.i2c_srv_cal = ros.ServiceProxy(self.i2c_srv_cal_name, Calibration)
        ros.logwarn("Check!\n")

        # Display calibration informations
        ros.loginfo("To calibrate the gyroscopes, place the robot motionless on a table for a few seconds.")
        ros.loginfo("To calibrate the magnetometer, perform an infinity pattern continuously continuously for a dozen time (no magnet in the surroundings)")
        ros.loginfo("To calibrate the accelerometers, hold the sensor in about 6 different positions for a few seconds.  Think of a cube and the 6 faces on it and try to slowly move the sensor between each face, holding it there for a few seconds.")
        ros.loginfo("The fusion algorithm will calibrate once all the sensors have been calibrated.")
        ros.loginfo("Press ENTER when calibration is finished (all status are 3).\n")

        # Display calibration status until finished
        while not ros.is_shutdown():
            if self.i2c_msg:
                system =  self.i2c_msg.cal_sys
                gyro = self.i2c_msg.cal_gyro
                accel = self.i2c_msg.cal_acc
                mag = self.i2c_msg.cal_mag
                sys.stdout.write('Algo cal = {0}  Gyro cal = {1}  Accel cal = {2}  Mag cal = {3}\r'.format(system, gyro, accel, mag))
                time.sleep(0.2)

            if self.k_list:
                if self.k_list[-1] == '\n':
                    self.k_list = []
                    break

        # Call save calibration service
        print("\n")
        data = json.dumps({})
        ack = self.i2c_srv_cal(data=data, filename=self.i2c_sens_filename)
        if ack.success:
            utils.save_calib_file(ack.calib, self.i2c_sens_filename)
            ros.loginfo("Calibrated with success!")
        else:
            ros.logerr("Calibration failed! Check out the log in the UART node")
        
        # Close and delete subscribers
        self.i2c_sub.unregister()
        del self.i2c_srv_cal
        del self.i2c_sub

    def calibrate_leg_sensors(self):

        # Wait for the uart node to be started, then subscribe to its topics
        print("\n === Legs Sensors Calibration ===\n")
        ros.logwarn("Waiting for the /tigrillo_io/uartd node to be started...")
        ros.wait_for_service(self.uart_srv_sens_cal_name)
        self.uart_sub = ros.Subscriber(self.uart_sub_name, Sensors, callback=self.__uart_ros_sub, queue_size=self.queue_size)
        self.uart_srv_cal = ros.ServiceProxy(self.uart_srv_sens_cal_name, Calibration)
        while not self.uart_msg:
            time.sleep(0.2)
        ros.logwarn("Check!\n")

        # Display calibration informations
        ros.loginfo("To calibrate the legs, move them manually to their minimal and maximal positions then press ENTER.")

        # Display calibration status until finished
        i = 0
        while True:
            BL = self.uart_msg.BL_raw
            BR = self.uart_msg.BR_raw
            FL = self.uart_msg.FL_raw
            FR = self.uart_msg.FR_raw
            sys.stdout.write('BL = {0} BR = {1} FL = {2} FR = {3}             \r'.format(BL, BR, FL, FR))

            if i == 0:
                BL_min = BL
                BL_max = BL
                BR_min = BR
                BR_max = BR
                FL_min = FL
                FL_max = FL
                FR_min = FR
                FR_max = FR
            else:
                if BL < BL_min:
                    BL_min = BL
                if BL > BL_max:
                    BL_max = BL
                if BR < BR_min:
                    BR_min = BR
                if BR > BR_max:
                    BR_max = BR
                if FL < FL_min:
                    FL_min = FL
                if FL > FL_max:
                    FL_max = FL
                if FR < FR_min:
                    FR_min = FR
                if FR > FR_max:
                    FR_max = FR

            time.sleep(0.01)
            i += 1 

            if  self.k_list:
                if  self.k_list[-1] == '\n':
                    self.k_list = []
                    break

        # Test the angle values
        print("\n")
        ros.loginfo("Test the angle values and press ENTER!")
        REAL_MAX = 135
        REAL_MIN = 30
        while True:
            BL = REAL_MAX - (REAL_MAX - REAL_MIN) * (self.uart_msg.BL - BL_min) / (BL_min - BL_max)
            BR = REAL_MAX - (REAL_MAX - REAL_MIN) * (self.uart_msg.BR - BR_min) / (BR_min - BR_max)
            FL = REAL_MAX - (REAL_MAX - REAL_MIN) * (self.uart_msg.FL - FL_min) / (FL_min - FL_max)
            FR = REAL_MAX - (REAL_MAX - REAL_MIN) * (self.uart_msg.FR - FR_min) / (FR_min - FR_max)
            sys.stdout.write('BL = {0} BR = {1} FL = {2} FR = {3}             \r'.format(BL, BR, FL, FR))

            time.sleep(0.01)

            if self.k_list:
                if  self.k_list[-1] == '\n':
                    self.k_list = []
                    break

        # Call save calibration service
        print("\n")
        data = json.dumps({"Real Max": REAL_MAX, "Real Min": REAL_MIN, "Back Left Max": BL_max,
                           "Back Left Min": BL_min, "Back Right Max": BR_max, "Back Right Max": BR_min, 
                           "Front Left Max": FL_max, "Front Left Min": FL_min, "Front Right Max": FR_max,
                           "Front Right Min": FR_min})
        ack = self.uart_srv_cal(data=data, filename=self.uart_sens_filename)
        if ack.success:
            utils.save_calib_file(data, self.uart_sens_filename)
            ros.loginfo("Calibrated with success!")
        else:
            ros.logerr("Calibration failed! Check out the log in the UART node")

        # Close and delete subscribers
        self.uart_sub.unregister()
        del self.uart_srv_cal
        del self.uart_sub

    def calibrate_actuators(self):

        # Wait for the uart node to be started
        print("\n === Legs Actuators Calibration ===\n")
        if not "/gazebo/model_states" in  [item[0] for item in ros.get_published_topics()]:
            ros.logwarn("You must start the gazebo node to realize this calibration. Stopping...")
            return
        ros.logwarn("Waiting for the /tigrillo_io/uartd node to be started...")
        ros.wait_for_service(self.uart_srv_sens_cal_name)
        ros.logwarn("...Check!")
        self.uart_pub = ros.Publisher(self.uart_pub_name, Motors, queue_size=self.queue_size)
        self.uart_srv_act_cal_save = ros.ServiceProxy(self.uart_srv_act_cal_save_name, Calibration)
        self.uart_srv_act_cal_load = ros.ServiceProxy(self.uart_srv_act_cal_load_name, Calibration)
        ros.loginfo("In gazebo, make sure the calibration world is loaded")
        ros.loginfo("Press 1 to increase the front position of the robot actuators, 2 to decrease it. " 
                    "Press 3 to increase the back position of the robot actuators, 4 to decrease it. "
                    " Press ENTER when finished!")
        
        # Create a timer to ensure synchro with real-time
        t = timing.Timer(real_time=True, runtime=50, dt=0.01)
        t.start()

        while not t.is_finished():

            sine = math.pi / 5 * math.sin(3 * t.st)
            self.uart_pub.publish(run_time=t.st, FL=sine, FR=sine, BL=sine, BR=sine)

            if self.k_list:
                if self.k_list[-1] == '1\n':
                    print "Increase front position"
                    self.k_list = []
                    ack = self.uart_srv_act_cal_load(data="", filename=self.uart_act_filename)
                    val = json.loads(ack.calib)
                    val["offset"] += 5
                    self.uart_srv_act_cal_save(data=json.dumps(val), filename=self.uart_act_filename)

                elif self.k_list[-1] == '2\n':
                    print "Decrease front position"
                    self.k_list = []
                    ack = self.uart_srv_act_cal_load(data="", filename=self.uart_act_filename)
                    val = json.loads(ack.calib)
                    val["offset"] -= 5
                    self.uart_srv_act_cal_save(data=json.dumps(val), filename=self.uart_act_filename)

                elif self.k_list[-1] == '3\n':
                    print "Increase back position"
                    self.k_list = []
                    ack = self.uart_srv_act_cal_load(data="", filename=self.uart_act_filename)
                    val = json.loads(ack.calib)
                    val["offset"] -= 2.5
                    val["mul_factor"] += 25 / (2 * math.pi)
                    self.uart_srv_act_cal_save(data=json.dumps(val), filename=self.uart_act_filename)

                elif self.k_list[-1] == '4\n':
                    print "Decrease back position"
                    self.k_list = []
                    ack = self.uart_srv_act_cal_load(data="", filename=self.uart_act_filename)
                    val = json.loads(ack.calib)
                    val["offset"] += 2.5
                    val["mul_factor"] -= 25 / (2 * math.pi)
                    self.uart_srv_act_cal_save(data=json.dumps(val), filename=self.uart_act_filename)

                elif self.k_list[-1] == '\n':
                    self.k_list = []
                    break

            # Update timers and pause if needed
            t.update()

    def __wait_for_key(self):

        while True:
            a = sys.stdin.readline()
            self.k_list.append(a)

    def __uart_ros_sub(self, msg):

        self.uart_msg = msg

    def __i2c_ros_sub(self, msg):

        self.i2c_msg = msg

    def start(self):

        ros.init_node(self.node_name, log_level=ros.INFO)

        # Start keyboard thread
        thread.start_new_thread(self.__wait_for_key, ())


      
