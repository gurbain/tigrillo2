import serial
import rospy as ros
from std_msgs.msg import Float64
from scipy.interpolate import interp1d

# Init
ros.init_node('magnet_sensor', anonymous=True)
p_FR = ros.Publisher('magnet_FR', Float64, queue_size=1)
p_FL = ros.Publisher('magnet_FL', Float64, queue_size=1)
p_BR = ros.Publisher('magnet_BR', Float64, queue_size=1)
p_BL = ros.Publisher('magnet_BL', Float64, queue_size=1)
ser = serial.Serial(port='/dev/ttyACM0', baudrate=9600)
print("connected to: " + ser.portstr)
count = 0
word = ""
vals = []

# Calib data

a_fl = [40, 44, 50, 60, 70, 80, 90, 100, 110, 120, 180]
fl = [0, 368, 1391, 2353, 2714, 2878, 2954, 2991, 3013, 3028, 3200]
a_fr = [40, 56, 60, 70, 80, 90, 100, 110, 120, 180]
fr = [0, 1327, 1625, 2322, 2681, 2851, 2931, 2974, 2995, 3200]
a_bl = [50, 56, 60, 65, 70, 80, 90, 100, 110, 120, 180]
bl = [0, 131, 545, 1602, 2113, 2644, 2828, 2928, 2981, 3008, 3200]
a_br = [40, 45, 50, 60, 70, 80, 90, 100, 110, 120, 180]
br = [0, 130, 330, 1938, 2496, 2765, 2884, 2946, 2976, 2994, 3200]
f_fr = interp1d(fr, a_fr, kind='cubic')
f_fl = interp1d(fl, a_fl, kind='cubic')
f_br = interp1d(br, a_br, kind='cubic')
f_bl = interp1d(bl, a_bl, kind='cubic')

# Loop
while not ros.is_shutdown():
    for read in ser.read():
        ch = str(read)
        if ch != "," and ch !=";":
            word += ch
        if ch == ",":
            try:
                vals.append(int(word))
            except ValueError:
                pass
            word = ""

        if ch == ";":
            try:
                vals.append(int(word))
            except ValueError:
                pass
            word = ""
            try:
                if len(vals) == 4:
                    p_FR.publish(data=f_fr(vals[0]))
                    p_FL.publish(data=f_fl(vals[1]))
                    p_BR.publish(data=f_br(vals[2]))
                    p_BL.publish(data=f_bl(vals[3]))
            except ValueError:
                pass
            vals = []

# Close
ser.close()