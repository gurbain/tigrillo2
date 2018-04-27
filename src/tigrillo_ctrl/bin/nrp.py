#! /usr/bin/env python

import os
import pickle
import psutil
import rospy as ros
import sys
import time

import tigrillo_ctrl
from tigrillo_ctrl import utils, control, timing, ctrl_ros


__author__ = "Gabriel Urbain" 
__copyright__ = "Copyright 2017, Human Brain Projet, SP10"

__license__ = "MIT" 
__version__ = "2.0" 
__maintainer__ = "Gabriel Urbain"
__email__ = "gabriel.urbain@ugent.be" 
__status__ = "Research" 
__date__ = "April 10th, 2018"

PKL_FILE = "/home/gabs48/src/quadruped/tigrillo2/src/tigrillo_ctrl/nrp/bounding.pkl"


if __name__ == '__main__':

    # Create config file from Alex pkl
    with open(PKL_FILE, "rb") as f:
        config = pickle.load(f)

    print config

    # Create and configure robot
    rob = ctrl_ros.CTRLROS()
    rob.start()

    # Create a controller with config file
    ctl = control.CPG(config)

    # Initialise the other nodes
    rob.set_sensors_frequency(int(1 / float(config["Controller"]["time_step"])))

    # Create a timer to ensure synchro with real-time
    t = timing.Timer(real_time=True, runtime=float(config["Controller"]["runtime"]), dt=float(config["Controller"]["time_step"]))
    t.start()

    # Perform experiment
    while not t.is_finished():

        # Produce actuators control signal
        command = ctl.step(t.st)
        i = 0
        for c in command:
            command[i] = c
            i += 1

        # Update the robot actuators
        rob.update_actuators(command, t.st)

        # Get the last sensors
        measure = rob.get_last_sensors()

        # Update timers and pause if needed
        t.update()


    # Terminate experiment
    t.print_info()
