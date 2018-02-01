
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
from std_srvs.srv import Empty
from tigrillo_ctrl.msg import Sensors, Motors, Imu

__author__ = "Gabriel Urbain" 
__copyright__ = "Copyright 2017, Human Brain Projet, SP10"

__license__ = "MIT" 
__version__ = "2.0" 
__maintainer__ = "Gabriel Urbain"
__email__ = "gabriel.urbain@ugent.be" 
__status__ = "Research" 
__date__ = "January 26th, 2018"


class Gazebo(threading.Thread):

    def __init__(self, model="tigrillo.world"):

        threading.Thread.__init__(self)

        self.sim_ps = None
        self.sim_ps_name = "rosrun"
        self.sim_package = "gazebo_ros"
        self.sim_node = "gzserver"
        self.sim_physics = "gzserver"
        self.sim_model = model

        self.reset_sim_service = '/gazebo/reset_simulation'
        self.pause_sim_service = '/gazebo/pause_physics'
        self.unpause_sim_service = '/gazebo/unpause_physics'
        self.motor_pub_name = 'tigrillo_rob/uart_actuators'
        self.sensor_sub_name = 'tigrillo_rob/sim_sensors'

        self.sub_clock = None
        self.sub_state = None
        self.sim_duration = 0
        self.sensors = {"time": 0, "FL": 0, "FR": 0, "BL": 0, "BR": 0}
 
        self.daemon = True

    def run(self):

        # Configure ROS node
        self.reset_sim_proxy = ros.ServiceProxy(self.reset_sim_service, Empty)
        self.pause_sim_proxy = ros.ServiceProxy(self.pause_sim_service, Empty)
        self.unpause_sim_proxy = ros.ServiceProxy(self.unpause_sim_service, Empty)
        self.motor_pub = ros.Publisher(self.motor_pub_name, Motors, queue_size=1)
        self.sub_clock = ros.Subscriber("/clock", Clock, callback=self._reg_sim_duration, queue_size=1)
        self.sensor_sub = ros.Subscriber(self.sensor_sub_name, Sensors, callback=self._reg_sensors, queue_size=1)

        self.start_gazebo()

    def stop(self):

        self.stop_gazebo()

    def start_gazebo(self):

        self.sim_duration = 0

        self.sim_status = self.get_gazebo_status()
        if self.sim_status == psutil.STATUS_RUNNING or self.sim_status == psutil.STATUS_SLEEPING:
            print("Simulation is already started")
            return

        proc = [self.sim_ps_name, self.sim_package, self.sim_node, self.sim_model]
        sys.stdout.write("Starting gzserver.. ") 
        sys.stdout.flush()# with params: " + str(proc)
        self.sim_ps = subprocess.Popen(proc, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        proc_out, proc_err = self.sim_ps.communicate()

    def stop_gazebo(self):

        sys.stdout.write("Stoping gzserver!\t") 
        sys.stdout.flush()
        tmp = os.popen("ps -Af").read()
        gzserver_count = tmp.count(self.sim_physics)

        if gzserver_count > 0:
            os.system("killall -9 " + self.sim_physics)

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

    def _reg_sim_duration(self, time):

        self.sim_duration = time.clock.secs + time.clock.nsecs/1000000000.0

    def _reg_sensors(self, msg):

        self.sensors = {"FR": msg.FR, "FL": msg.FL, "BR": msg.BR, "BL": msg.BL, "time": msg.run_time}

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

        # Check time for 10 seconds
        for j in range(11):
            print("Time: " + str(j) + "s and sim time: " + str(p.get_gazebo_time()) + \
                  "s and status: " + str(p.get_gazebo_status()))
            if ros.is_shutdown():
                p.stop()
                exit(-1)
            time.sleep(1)
        
        p.stop()