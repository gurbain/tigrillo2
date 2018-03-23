import json
import numpy as np
from scipy.interpolate import interp1d
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

CALIB_FOLDER = "/home/gabs48/src/quadruped/tigrillo2/data/calibrations/"

def get_style_colors():

	if 'axes.prop_cycle' in plt.rcParams:
		cols = plt.rcParams['axes.color_cycle']
	else:
		cols = ['b', 'r', 'y', 'g', 'k']
	return cols

a_fl = [40, 44, 50, 60, 70, 80, 90, 100, 110, 120, 180]
fl = [0, 368, 1391, 2353, 2714, 2878, 2954, 2991, 3013, 3028, 3200]
a_fr = [40, 56, 60, 70, 80, 90, 100, 110, 120, 180]
fr = [0, 1327, 1625, 2322, 2681, 2851, 2931, 2974, 2995, 3200]
a_bl = [52, 56, 60, 65, 70, 80, 90, 100, 110, 120, 180]
bl = [0, 131, 545, 1602, 2113, 2644, 2828, 2928, 2981, 3008, 3200]
a_br = [40, 45, 50, 60, 70, 80, 90, 100, 110, 120, 180]
br = [0, 130, 330, 1938, 2496, 2765, 2884, 2946, 2976, 2994, 3200]
f_fr = interp1d(fr, a_fr, kind='cubic')
f_fl = interp1d(fl, a_fl, kind='cubic')
f_br = interp1d(br, a_br, kind='cubic')
f_bl = interp1d(bl, a_bl, kind='cubic')
x2 = np.linspace(0, 3100)

plt.plot(fr, a_fr, "*", linewidth=5, color=get_style_colors()[0])
plt.plot(x2, f_fr(x2), ":", linewidth=2.5, color=get_style_colors()[0], label="Front Right Interpolation")
plt.plot(fl, a_fl, "*", linewidth=5, color=get_style_colors()[1])
plt.plot(x2, f_fl(x2), "-.", linewidth=2.5, color=get_style_colors()[1], label="Front Left Interpolation")
plt.plot(br, a_br, "*", linewidth=5, color=get_style_colors()[2])
plt.plot(x2, f_br(x2), "-", linewidth=2.5, color=get_style_colors()[2], label="Back Right Interpolation")
plt.plot(bl, a_bl, "*", linewidth=5, color=get_style_colors()[3])
plt.plot(x2, f_bl(x2), "--", linewidth=2.5, color=get_style_colors()[3], label="Back Left Interpolation")

plt.title("Magnet Sensor Transfer Functions")
plt.xlabel("Sensor values")
plt.ylabel('Knee angles')
plt.ylim([30, 140])
plt.legend(loc="upper left", fontsize="small")
plt.savefig("calib.png", format='png', dpi=300)

from matplotlib2tikz import save as tikz_save
tikz_save('calib.tex', figureheight='6cm', figurewidth='7.33cm')
plt.close()


with open(CALIB_FOLDER + "calib_uart_sens.json", 'w') as cal_file:
    data = {"a_fl": a_fl, "fl":fl, "a_fr": a_fr, "fr":fr, "bl":bl, "a_bl": a_bl, "a_br": a_br, "br":br}
    cal_file.write(json.dumps(data))