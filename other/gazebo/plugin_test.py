#! /usr/bin/env python

import json
import math
import rospy as ros
import time

from std_msgs.msg import String

# Create node
ros.init_node('tigrillo_ctrl_test', anonymous=True)
pub = ros.Publisher('tigrillo_ctrl/uart_actuators', String, queue_size=1)


t_max = 10
t = 0
dt = 0.01

# In a loop, send a actuation signal
while not ros.is_shutdown() and t < t_max:
	
	val = {"FL": -math.pi / 4 * math.sin(t), "FR": math.pi / 4 * math.sin(t), \
		   "BL": math.pi / 4 * math.sin(t),  "BR": -math.pi / 4 * math.sin(t)}

	ros.loginfo("Publishing in tigrillo_ctrl/uart_actuators topic: " + str(val))
	pub.publish(json.dumps(val))
	
	time.sleep(dt)
	t += dt