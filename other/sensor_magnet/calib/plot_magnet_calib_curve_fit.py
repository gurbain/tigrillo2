import numpy as np
from scipy.optimize import curve_fit


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

def func(x, a, b, c, d):
	return a * np.power(c * x-b, 3) + d
	#return a * np.exp(-b * x) + c
	#return a + b * np.power(1/(x - c), 1.5)
    #return a * np.exp(b * x) + c
    #return c - np.power((b / (x - a)), (2/3))/d

angles = [45, 50, 60, 70, 80, 90, 100, 110, 120]
fl = [870, 1540, 2435, 2807, 2923, 3002, 3040, 3065, 3077]

x = np.array(fl)
y = np.array(angles)

plt.plot(x, y, "*", linewidth=2, color=get_style_colors()[0], label="Samples")

print x, y
g_opt, g_cov = curve_fit(func, x, y)#) method="lm", maxfev=100000)
print g_opt, g_cov
x2 = np.linspace(200, 3500)
print func(x2, *g_opt)

plt.plot(x2, func(x2, *g_opt), "--", linewidth=1, color=get_style_colors()[1], label="Fitted Curve")

plt.title("Fitted Curve Transfer Function")
plt.xlabel("Sensor value")
plt.ylabel('Knee angle')
plt.ylim([30, 140])
plt.legend(loc="upper left", fontsize="x-small")
plt.savefig("magnet_polyfit.png", format='png', dpi=300)
plt.close()