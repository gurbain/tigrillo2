import numpy as np
from scipy.interpolate import interp1d
import pickle


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

angles = [40, 45, 50, 60, 70, 80, 90, 100, 110, 120, 180]
fl = [0, 870, 1540, 2435, 2807, 2923, 3002, 3040, 3065, 3077, 3200]

x = np.array(fl)
y = np.array(angles)

plt.plot(x, y, "*", linewidth=2, color=get_style_colors()[0], label="Samples")

f = interp1d(x, y)
f2 = interp1d(x, y, kind='cubic')
f3 = interp1d(x, y, kind='quadratic')
f_dict = {}            
for key in f2.__dict__.keys():
    if key != '_function' and key!= 'norm':
        f_dict[key] = f2.__getattribute__(key)

print f_dict   

print pickle.dumps(f_dict)


x2 = np.linspace(0, 3200)
#plt.plot(x2, f(x2), "--", linewidth=1, color=get_style_colors()[1], label="Linear Interpolation")
plt.plot(x2, f2(x2), "--", linewidth=1, color=get_style_colors()[2], label="Cubic Interpolation")
#plt.plot(x2, f3(x2), "--", linewidth=1, color=get_style_colors()[3], label="Quadratic Interpolation")

plt.title("Interpolated Transfer Function")
plt.xlabel("Sensor value")
plt.ylabel('Knee angle')
plt.ylim([30, 140])
plt.legend(loc="upper left", fontsize="x-small")
plt.savefig("magnet_interpol.png", format='png', dpi=300)
plt.close()