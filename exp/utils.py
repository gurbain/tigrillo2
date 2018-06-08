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

import copy
import math
import numpy as np
import os
from os.path import expanduser
import pickle
import psutil
from scipy.signal import butter, filtfilt
from shutil import copyfile
import time


def cleanup():

    print('\n\n -- Quitting and killing all children processes! -- \n')
    process = psutil.Process()
    children = process.children(recursive=True)
    time.sleep(0.2)
    for p in children:
        p.kill()
    process.kill()


def mkdir(path):
    """ Create a directory if it does not exist yet """

    if not os.path.exists(path):
        os.makedirs(path)


def cp(src, dst):
    """ Copy a file in another """

    copyfile(src, dst)


def timestamp():
    """ Return a string stamp with current date and time """

    return time.strftime("%Y%m%d-%H%M%S", time.localtime())


def var_name(var):
    """ Return the name of a variable """
    for name,value in globals().items() :
        if value is var :
            return name
    return None 


def plot(y, x=None, save_folder=None):
    """ Simple tools to plot and save a figure quickly"""

    if save_folder is None:
        save_folder = expanduser("~") + "/"

    name = var_name(y)

    if name is None:
        filename = timestamp()
        label = "Unknown"
    else:
        filename = name + timestamp()
        label = name

    if x is not None:
        plt.plot(x, y, linewidth=1, label=label)
    else:
        plt.plot(y, linewidth=1, label=label)
    plt.savefig(save_folder + filename + ".png", format='png', dpi=300)
    plt.close()


def filter_duplicate(mylist):

    l = np.array(mylist)
    old_t = [d["time"] for d in l]
    t, i = np.unique(old_t, return_index=True)
    fl = [d["FL"] for d in l[i]]
    fr = [d["FR"] for d in l[i]]
    bl = [d["BL"] for d in l[i]]
    br = [d["BR"] for d in l[i]]

    return t, fl, fr, bl, br


def mse(arr1, arr2):
    """ Compute MSE between two arrays (1D) """

    assert arr1.shape == arr2.shape, "Mean Square Error can only be computed on matrices with same size"
    a, b = np.matrix(arr2).shape
    return np.sum(np.square(arr2 - arr1)) / float(a * b)


def nrmse(arr1, arr2):
    """ Compute NRMSE between two arrays (1D) """

    # Center signals around 0?
    rmse = np.sqrt(mse(arr1, arr2))
    max_val = max(np.max(arr1), np.max(arr2))
    min_val = min(np.min(arr1), np.min(arr2))

    return (rmse / (max_val - min_val))


def abse_t_inv(t, max_t, fl_r, fr_r, bl_r, br_r, fl_s, fr_s, bl_s, br_s):
    
    t_step = t[1] - t[0]
    t_range = t[-1] - t[0]
    t_inc = int(max_t / t_step)
    score = 100000
    for i in range(-t_inc, t_inc):
        fl_nrmse = np.trapz(np.abs(fl_r - np.roll(fl_s, i)), dx=t_step)
        fr_nrmse = np.trapz(np.abs(fr_r - np.roll(fr_s, i)), dx=t_step)
        bl_nrmse = np.trapz(np.abs(bl_r - np.roll(bl_s, i)), dx=t_step)
        br_nrmse = np.trapz(np.abs(br_r - np.roll(br_s, i)), dx=t_step)
        score = min(score, (fl_nrmse + fr_nrmse + bl_nrmse + br_nrmse) / 4)

    return score


def corr(arr1, arr2):
    """ Compute Average Pearson correlation for arrays of sgnals """

    assert arr1.shape == arr2.shape, "Correlation can only be computed on matrices with same size"
    a, b = arr2.shape
    c = []
    for i in range(a):
        c.append(np.corrcoef(arr1[i,:], arr2[i,:])[1, 0])

    # Recenter in interval 0, 1 where 0 is totally correlated, 0.5 uncorrelated and 1 totally in opposition
    d = -(sum(c)/len(c))/2 + 0.5

    return d


def corr_nrmse(arr1, arr2):
    """ Compute NRMSE and multiply by pearson correlation (and then average for multiple signals). """

    assert arr1.shape == arr2.shape, "Correlation can only be computed on matrices with same size"
    a, b = arr2.shape
    c = []
    n = []
    v = []
    for i in range(a):
        n.append(nrmse(np.matrix(arr1[i,:]), np.matrix(arr2[i,:])))
        c.append(np.corrcoef(arr1[i,:], arr2[i,:])[1, 0])
        v.append((- c[i]/2 * (n[i] - 0.5)) + 0.5)

    d = sum(v) / len(v)

    return d


def unorm(normed_val, minimums, maximums):

    assert len(normed_val) == len(minimums), "The value array should be the same size as the extremums"
    assert len(normed_val) == len(maximums), "The value array should be the same size as the extremums"

    val = []
    for i, v in enumerate(normed_val):
        val.append(minimums[i] + v * (maximums[i] - minimums[i]))

    return val


def save_on_top(newdata, filename):
    """ Append data to a file that is already saved """

    if os.path.exists(filename):
        data = pickle.load(open(filename, "rb"))
        final = list(copy.copy(data)) + [copy.copy(newdata)]
    else:
        final = [copy.copy(newdata)]

    pickle.dump(final, open(filename, "wb"), protocol=2)


def center_norm(sig):

    sig_max = np.max(sig)
    sig_min = np.min(sig)
    sig = (sig - sig_min) / (sig_max - sig_min)
    return sig - 0.5


def center_norm_2(sig_drive, sig_follow):

    sig_max = np.max(sig_drive)
    sig_min = np.min(sig_drive)
    sig_drive = (sig_drive - sig_min) / (sig_max - sig_min)
    sig_follow = (sig_follow - sig_min) / (sig_max - sig_min)
    return [sig_drive - 0.5, sig_follow - 0.5]


def divide_periodic(f, x, y):

    l = y.size
    ts = x[1] - x[0]
    t = x[-1] - x[0]
    n = t / ts
    r = int(np.floor(t * f))
    c = int(np.floor(l / r))

    y_new = np.empty([r, c])
    x_new = x[0:c]

    for i in range(r):
        j = int(np.floor(i * c))
        if j + c < l:
            y_new[i, :] = y[j: j+c]
        else:
            j = int(np.floor((i-1) * c))
            print "overloading"
            y_new[i, :] = y[j: j+c]
    y_new = np.transpose(y_new)

    return x_new, y_new


def zero_crossing(sig, t):

    b, a = butter(2, 0.03, analog=False)
    sig_filtered = filtfilt(b, a, sig)

    zeros = []
    i = 0
    p_val = None
    p_dev = None
    for a in center_norm(sig_filtered):
        if i == 0:
            p_val = a
        elif i == 1:
            p_dev = a - p_val
            p_val = a
        else:
            if (a * p_val) < 0:
                if p_dev > 0:
                    zeros.append(t[i])
            p_dev = a - p_val
            p_val = a
        i += 1

    return zeros

def quaternion_to_euler_angle(w, x, y, z):
    ysqr = y * y
    
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = math.degrees(math.atan2(t0, t1))
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = math.degrees(math.atan2(t3, t4))
    
    return X, Y, Z