#! /usr/bin/env python

import json
import math
import rospy as ros
import time

from std_msgs.msg import String

# Create node
ros.init_node('tigrillo_ctrl_test', anonymous=True)
pub = ros.Publisher('tigrillo_ctrl/uart_actuators', String, queue_size=1)

t = 0
dt = 0.05
t_max = 10

# In a loop, send a actuation signal
while not ros.is_shutdown() and t < t_max:
	
	val = {"FL": -math.pi / 8 * math.sin(dt * t), "FR": math.pi / 8 * math.sin(dt * t),
		   "BL": math.pi / 8 * math.sin(dt * t), "BR": -math.pi / 8 * math.sin(dt * t)}

	ros.loginfo("Publishing in tigrillo_ctrl/uart_actuators topic: " + str(val))
	pub.publish(json.dumps(val))
	
	time.sleep(dt)
	t += dt