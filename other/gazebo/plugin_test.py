#! /usr/bin/env python

import json
import math
import rospy as ros
import time

from std_msgs.msg import String
from tigrillo_ctrl.msg import Sensors, Motors, Imu

# Create node
ros.init_node('tigrillo_ctrl_test', anonymous=True)
pub = ros.Publisher('tigrillo_rob/uart_actuators', Motors, queue_size=1)


t_max = 50
t = 0
dt = 0.01

# In a loop, send a actuation signal
while not ros.is_shutdown() and t < t_max:
	
	FL = -math.pi / 2 * math.sin(t)
	FR = math.pi / 2 * math.sin(t)
	BL = math.pi / 2 * math.sin(t)
	BR = -math.pi / 2 * math.sin(t)

	pub.publish(FL=FL, FR=FR, BL=BL, BR=BR)
	
	time.sleep(dt)
	t += dt