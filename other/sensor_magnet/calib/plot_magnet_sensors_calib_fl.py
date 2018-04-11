import numpy as np
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

angles = [45, 50, 60, 70, 80, 90, 100, 110, 120]
fl = [870, 1540, 2435, 2807, 2923, 3002, 3040, 3065, 3077]

x = np.array(fl)
y = np.array(angles)


plt.plot(x, y, "*", linewidth=2, color=get_style_colors()[0], label="Samples")

for i in range(3, 7):
	a = np.polyfit(x, y, i)
	g = np.poly1d(a)
	x2 = np.linspace(200, 3500)
	plt.plot(x2, g(x2), "--", linewidth=1, color=get_style_colors()[i%5], label="Pol-" + str(i) + " interpolation")
plt.title("Front Left Magnet Sensor Transfer Function")
plt.xlabel("Sensor value")
plt.ylabel('Knee angle')
plt.ylim([30, 140])
plt.legend(loc="upper left", fontsize="x-small")
plt.savefig("magnet_polyfit.png", format='png', dpi=300)
plt.close()