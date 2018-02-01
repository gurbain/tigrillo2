import copy
import numpy as np
import os
import pickle
import psutil
from shutil import copyfile
import time


def cleanup():

    print('\n\n -- Interrupted by User or Error! Killing all children processes -- \n')
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
    return np.sum((arr2 - arr1) ** 2) / float(a * b)


def nrmse(arr1, arr2):
    """ Compute NRMSE between two arrays (1D) """

    # Center signals around 0?
    
    rmse = np.sqrt(mse(arr1, arr2))
    max_val = max(np.max(arr1), np.max(arr2))
    min_val = min(np.min(arr1), np.min(arr2))

    return (rmse / (max_val - min_val))

def save_on_top(newdata, filename):
    """ Append data to a file that is already saved """

    if os.path.exists(filename):
        data = pickle.load(open(filename, "rb"))
        final = list(copy.copy(data)) + [copy.copy(newdata)]
    else:
        final = [copy.copy(newdata)]

    pickle.dump(final, open(filename, "wb"), protocol=2)