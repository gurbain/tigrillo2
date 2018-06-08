
import logging
import numpy as np
import os
import psutil
import rospy as ros
import subprocess
import sys
import threading
import time

from rosgraph_msgs.msg import Clock
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Empty
from gazebo_msgs.srv import AdvanceSimulation
from tigrillo_2_plugin.msg import Sensors, Motors
from sensor_msgs.msg import Imu

import utils

__author__ = "Gabriel Urbain" 
__copyright__ = "Copyright 2017, Human Brain Projet, SP10"

__license__ = "MIT" 
__version__ = "2.0" 
__maintainer__ = "Gabriel Urbain"
__email__ = "gabriel.urbain@ugent.be" 
__status__ = "Research" 
__date__ = "January 26th, 2018"


class Gazebo(threading.Thread):

    def __init__(self, model="tigrillo.world", view=False):

        threading.Thread.__init__(self)

        self.sim_ps = None
        self.sim_ps_name = "rosrun"
        self.sim_package = "gazebo_ros"
        if view:
            self.sim_node = "gazebo"
            self.sim_ps_list = ["gzserver", "gzclient"]
        else:
            self.sim_node = "gzserver"
            self.sim_ps_list = ["gzserver"]
        self.sim_model = model

        self.reset_sim_service = '/gazebo/reset_simulation'
        self.pause_sim_service = '/gazebo/pause_physics'
        self.unpause_sim_service = '/gazebo/unpause_physics'
        self.step_sim_service = "/gazebo/advance_simulation"
        self.pose_sub_name = '/gazebo/model_states'
        self.clock_sub_name = '/clock'
        self.motor_pub_name = '/tigrillo_rob/uart_actuators'
        self.sensor_sub_name = '/tigrillo_rob/sim_sensors'
        self.motor_sub_name = '/tigrillo_rob/sim_motors'
        self.imu_sub_name = '/imu_data'
        self.ori_pub_name = '/tigrillo_rob/orientation'

        self.sub_clock = None
        self.sub_state = None
        self.sim_duration = 0
        self.sensors = {"time": 0, "FL": 0, "FR": 0, "BL": 0, "BR": 0}
        self.motors = {"time": 0, "FL": 0, "FR": 0, "BL": 0, "BR": 0}
        self.imu = {"ori_x": 0, "ori_y": 0, "ori_z": 0, "ori_w": 1}
        self.pose = {"x": 0, "y": 0, "z": 0, "a_x": 0, "a_y": 0, "a_z": 0, "a_w": 0}
 
        self.daemon = True

    def run(self):

        # Configure ROS node
        self.reset_sim_proxy = ros.ServiceProxy(self.reset_sim_service, Empty)
        self.pause_sim_proxy = ros.ServiceProxy(self.pause_sim_service, Empty)
        self.unpause_sim_proxy = ros.ServiceProxy(self.unpause_sim_service, Empty)
        self.step_sim_proxy = ros.ServiceProxy(self.step_sim_service, AdvanceSimulation)
        self.motor_pub = ros.Publisher(self.motor_pub_name, Motors, queue_size=1)
        self.sub_clock = ros.Subscriber(self.clock_sub_name, Clock, callback=self._reg_sim_duration, queue_size=1)
        self.sensor_sub = ros.Subscriber(self.sensor_sub_name, Sensors, callback=self._reg_sensors, queue_size=1)
        self.motor_sub = ros.Subscriber(self.motor_sub_name, Motors, callback=self._reg_motors, queue_size=1)
        self.imu_sub = ros.Subscriber(self.imu_sub_name, Imu, callback=self._reg_imu, queue_size=1)
        self.pose_sub = ros.Subscriber(self.pose_sub_name, ModelStates, callback=self._reg_pose, queue_size=1)
        # self.ori_pub = ros.Publisher(self.ori_pub_name, Float32MultiArray, queue_size=1)
        self.start_gazebo()

    def stop(self):

        self.stop_gazebo()

    def start_gazebo(self):

        self.sim_duration = 0

        self.sim_status = self.get_gazebo_status()
        if self.sim_status == psutil.STATUS_RUNNING or self.sim_status == psutil.STATUS_SLEEPING:
            sys.stdout.write("Already started!  ")
            return

        proc = [self.sim_ps_name, self.sim_package, self.sim_node, self.sim_model]
        sys.stdout.write("Starting gzserver.. ") 
        sys.stdout.flush()# with params: " + str(proc)
        self.sim_ps = subprocess.Popen(proc, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        proc_out, proc_err = self.sim_ps.communicate()

    def stop_gazebo(self):

        sys.stdout.write("Stoping gzserver!\t") 
        sys.stdout.flush()
        for p in self.sim_ps_list:
            tmp = os.popen("ps -Af").read()
            ps_count = tmp.count(p)
            if ps_count > 0:
                os.system("killall -9 " + p)

    def reset_gazebo(self):

        ros.wait_for_service(self.reset_sim_service)
        try:
            self.reset_sim_proxy()
        except ros.ServiceException as e:
            print("Reset simulation service call failed with error" + str(e))

    def pause_gazebo(self):

        ros.wait_for_service(self.pause_sim_service)
        try:
            self.pause_sim_proxy()
        except ros.ServiceException as e:
            print("Pause simulation service call failed with error" + str(e))

    def unpause_gazebo(self):

        ros.wait_for_service(self.unpause_sim_service)
        try:
            self.unpause_sim_proxy()
        except ros.ServiceException as e:
            print("Unpause simulation service call failed with error" + str(e))

    def step_gazebo(self, timestep):

        #ros.wait_for_service(self.step_sim_service)
        try:
            self.step_sim_proxy(timestep)
        except ros.ServiceException as e:
            print("Simulation step service call failed with error" + str(e))

    def get_gazebo_status(self):

        self.sim_status = psutil.STATUS_STOPPED

        for proc in psutil.process_iter():
            if proc.name() == self.sim_ps_name or proc.name() == self.sim_node:
                self.sim_status = proc.status()
                self.sim_pid = proc.pid

        return self.sim_status

    def get_gazebo_time(self):

        return self.sim_duration

    def get_sensors(self):

        return self.sensors

    def get_motors(self):

        return self.motors

    def get_imu(self):

        return self.imu

    def get_pose(self):

        return self.pose

    def _reg_sim_duration(self, time):

        self.sim_duration = time.clock.secs + time.clock.nsecs/1000000000.0

    def _reg_sensors(self, msg):

        self.sensors = {"FR": msg.FR, "FL": msg.FL, "BR": msg.BR, "BL": msg.BL, "time": msg.run_time}

    def _reg_motors(self, msg):

        self.motors = {"FR": msg.FR, "FL": msg.FL, "BR": msg.BR, "BL": msg.BL, "time": msg.run_time}

    def _reg_imu(self, msg):

        x, y, z = utils.quaternion_to_euler_angle(msg.orientation.w, msg.orientation.x, 
                                                  msg.orientation.y, msg.orientation.z)

        self.imu = {"ori_x": x, "ori_y": y, "ori_z": z}

        # self.ori_pub.publish(Float32MultiArray(data=[x, y, z]))
    
    def _reg_pose(self, msg):

        index = -1
        for i, name in enumerate(msg.name):
            if name == "tigrillo":
                index = i

        if index != -1:
            p = msg.pose[index].position
            o = msg.pose[index].orientation
            self.pose = {"x": p.x, "y": p.y, "z": p.z,
                        "a_x": o.x, "a_y": o.y, "a_z": o.z, "a_w": o.w}

        return

    def is_sim_started(self):

        return self.sim_duration > 0

    def actuate(self, update):

        self.motor_pub.publish(run_time=self.sim_duration, FL=update[0], FR=update[1], 
                              BL=update[2], BR=update[3])


if __name__ == '__main__':


    ros.init_node('physics', anonymous=True) # The ROS node can only be created in the main thread!!
    
    for i in range(2):
        p = Gazebo()
        p.start()

        # Check time for 20 seconds
        for j in range(21):
            print("Time: " + str(j) + "s and sim time: " + str(p.get_gazebo_time()) + \
                  "s and status: " + str(p.get_gazebo_status()) + \
                  " and X position: " + str(p.get_pose()['x']))
            if ros.is_shutdown():
                p.stop()
                exit(-1)
            time.sleep(1)
        
        p.stop()