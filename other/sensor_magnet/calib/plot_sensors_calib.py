import matplotlib
matplotlib.use("Agg")
from matplotlib.mlab import *
import matplotlib.pyplot as plt
plt.style.use('fivethirtyeight')
plt.rc('text', usetex=True)
plt.rc('font', family='serif')
plt.rc('axes', facecolor='white')
plt.rc('savefig', facecolor='white')
plt.rc('figure', autolayout=True)

def get_style_colors():

	if 'axes.prop_cycle' in plt.rcParams:
		cols = plt.rcParams['axes.color_cycle']
	else:
		cols = ['b', 'r', 'y', 'g', 'k']
	return cols

angles = [30, 40, 50, 60, 70, 80, 90, 100, 110, 120]
fr = [2724, 2749, 2765, 2770, 2783, 2785, 2789, 2793, 2795, 2795]
fr_std = [15, 15, 15, 15, 15, 15, 15, 15, 15, 15]
fl = [3525, 3542, 3563, 3602, 3625, 3665, 3664, 3701, 3720, 3743]

plt.errorbar(angles, fr, yerr=fr_std, fmt='.-', ecolor=get_style_colors()[1], color=get_style_colors()[0], linewidth=1.5)
plt.title("Front Right Flex Sensor Transfer Function")
plt.xlabel('Knee angle')
plt.ylabel("Sensor value")
plt.savefig("front_right.png", format='png', dpi=300)
plt.close()

plt.errorbar(angles, fl, yerr=fr_std, fmt='.-', ecolor=get_style_colors()[1], color=get_style_colors()[0], linewidth=1.5)
plt.title("Front Left Flex Sensor Transfer Function")
plt.xlabel('Knee angle')
plt.ylabel("Sensor value")
plt.savefig("front_left.png", format='png', dpi=300)
plt.close()