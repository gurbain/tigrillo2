#! /usr/bin/env python

import math
import rospy as ros
import time
from tigrillo_plugin.msg import Motors, Sensors

# The sensor callback function
def process_rx(msg):
    print "Sensors received: FR = " + str(msg.FR) +  " FL = " + str(msg.FL) +  "BR = " + str(msg.BR) + " BL = " + str(msg.BL)



# Create node
t = 0
ros.init_node('test_tigrillo', anonymous=True)
pub = ros.Publisher("tigrillo_rob/uart_actuators", Motors, queue_size=1) # Motor joint in degrees
sub = ros.Subscriber("tigrillo_rob/sim_sensors", Sensors, callback=process_rx, queue_size=1) # Spring joint in degrees

# Init to 0 (stand still)
while t < 20000:

    pub.publish(run_time=t, FL=0.0, FR=0.0, BL=0.0, BR=0.0)
    time.sleep(0.001)
    t += 1

# # Wait a bit
# time.sleep(8)

# # In a loop, send a actuation signal
# while (not ros.is_shutdown()) and t < 20000:
    
#     sine = 20 * math.sin(0.003 * t)
#     pub.publish(run_time=t, FL=sine, FR=sine, BL=sine, BR=sine)
    
#     time.sleep(0.001)
#     t += 1