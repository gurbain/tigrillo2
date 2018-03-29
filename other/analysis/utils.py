import copy
import numpy as np
from numpy.fft import rfft
from numpy import argmax, mean, diff, log
from matplotlib.mlab import find
import os
from parabolic import parabolic
import pickle
import psutil
from scipy.signal import blackmanharris, fftconvolve, butter, filtfilt
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


def filter_duplicate(mylist):

    newlist = []
    for i in mylist:
        if len(newlist) >= 1:
            l = [d["time"] for d in newlist]
            if i["time"] not in l:
                newlist.append(i)
        else:
            newlist.append(i)

    return newlist


def mse(arr1, arr2):
    """ Compute MSE between two arrays (1D) """

    assert arr1.shape == arr2.shape, "Mean Square Error can only be computed on matrices with same size"
    a, b = arr2.shape
    return np.sum(np.square(arr2 - arr1)) / float(a * b)


def nrmse(arr1, arr2):
    """ Compute NRMSE between two arrays (1D) """

    # Center signals around 0?
    
    rmse = np.sqrt(mse(arr1, arr2))
    max_val = max(np.max(arr1), np.max(arr2))
    min_val = min(np.min(arr1), np.min(arr2))

    return (rmse / (max_val - min_val))


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


def freq(sig, fs):
    """
    Estimate frequency using harmonic product spectrum (HPS)
    from here: https://gist.github.com/endolith/255291
    """ 

    corr = fftconvolve(sig, sig[::-1], mode='full')
    corr = corr[len(corr)//2:]
    d = diff(corr)
    start = find(d > 0)[0]
    peak = argmax(corr[start:]) + start
    px, py = parabolic(corr, peak)
    return fs / px


def freq2(sig, fs):
    """
    Estimate frequency from peak of FFT
    """

    windowed = sig * blackmanharris(len(sig))
    f = rfft(windowed)
    i = argmax(abs(f))
    true_i = parabolic(log(abs(f)), i)[0]
    return fs * true_i / len(windowed)


def freq3(sig, fs):
    """
    Estimate frequency by counting zero crossings
    """

    indices = find((sig[1:] >= 0) & (sig[:-1] < 0))
    crossings = [i - sig[i] / (sig[i+1] - sig[i]) for i in indices]
    return fs / mean(diff(crossings))


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

    b, a = butter(4, 0.03, analog=False)
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