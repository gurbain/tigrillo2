
"""
This file contains all third-party methods
"""

import configparser
import csv
import datetime
import git
import logging
from mpi4py import MPI as mpi
import os
import platform
from shutil import copyfile
import subprocess
import time
import threading

import tigrillo_io

__author__ = "Gabriel Urbain" 
__copyright__ = "Copyright 2017, Human Brain Projet, SP10"

__license__ = "MIT" 
__version__ = "1.0" 
__maintainer__ = "Gabriel Urbain"
__email__ = "gabriel.urbain@ugent.be" 
__status__ = "Research" 
__date__ = "February 22nd, 2017"


# Project variables

ROS_QUEUE_SIZE = 1
DATA_FOLDER = os.path.dirname(tigrillo_io.__file__) + "/../../../../../data"
RESULTS_FOLDER = DATA_FOLDER + "/results/"
CONFIG_FOLDER = DATA_FOLDER + "/configs/"
CALIB_FOLDER = DATA_FOLDER + "/calibrations/"


# Files and IOs Utils

def mkdir(path):
    """ Create a directory if it does not exist yet """

    if not os.path.exists(path):
        os.makedirs(path)


def cp(src, dst):
    """ Copy a file in another """

    copyfile(src, dst)


def load_csv(path):
    """ Load data from a csv file """

    if not path.endswith('.csv'):
        raise ValueError('CSV files should end with .csv, but got %s instead' % path)

    with open(path, mode='r') as infile:
        reader = csv.DictReader(infile)
        result = {}
        for row in reader:
            for column, value in row.iteritems():
                result.setdefault(column, []).append(float(value))

    return result

def save_csv_row(d, path, index):
    """ Save a dictionnary into one row of a csv file """

    dict_keys = sorted(d.keys())

    # For the first time, use dictwriter
    if index == 0 or os.stat(path).st_size == 0:
        with open(path, 'wb') as f:
            w = csv.DictWriter(f, dict_keys)
            w.writeheader()
            w.writerow(d)

    else:
        # Read the header
        with open(path, 'rb') as f:
            r = csv.reader(f)
            csv_keys = next(r)

        # If there are some new keys, rewrite the header
        if len(set(dict_keys) - set(csv_keys)) > 0:

            with open(path) as inf, open('tmp', 'wb') as outf:
                r = csv.reader(inf)
                w = csv.writer(outf)
                for i, line in enumerate(r):
                    if i == 0:
                        #print csv_keys
                        csv_keys = csv_keys + sorted(set(dict_keys) - set(csv_keys))
                        #print csv_keys
                        w.writerow(csv_keys)
                    else:
                        w.writerow(line)
            os.remove(path)
            os.rename('tmp', path)

        # Populate the last line with sorted values
        with open(path, 'ab') as f:
            w = csv.writer(f)
            to_add = []
            for k in csv_keys:
                if k in dict_keys:
                    to_add.append(d[k])
                else:
                    to_add.append("")
            w.writerow(to_add)


def save_calib_file(data, filename):
    """ Save a calibration JSON file in the calibration folder """

    mkdir(CALIB_FOLDER)
    with open(CALIB_FOLDER + filename, 'w') as cal_file:
        cal_file.write(data)


def load_calib_file(filename):
    """ Load a calibration JSON file from the calibration folder """

    with open(CALIB_FOLDER + filename, 'r') as cal_file:
        data = json.load(cal_file)

    return data


def retrieve_config(arg_list, default_config):
    """ Retrieve a dictionary from a config file """

    if len(arg_list) > 1:
        config_file = arg_list[-1]
    else:
        config_file = default_config

    # Convert config file
    config = configparser.ConfigParser()
    config.read(config_file)
    
    return {s: dict(config.items(s)) for s in config.sections()}



def dict_keys_to_str(dictionary):
    """ Recursively converts dictionary keys to strings """

    if not isinstance(dictionary, dict):
        return dictionary
    return dict((str(k), dict_keys_to_str(v)) 
        for k, v in dictionary.items())


# Information printing utils

def timestamp():
    """ Return a string stamp with current date and time """

    return time.strftime("%Y%m%d-%H%M%S", time.localtime())


def get_pid_string():
    """ Return a string with process PID """

    return "PID: " + str(os.getpid())


def get_date_string():
    """ Return a string with datetime """

    return "Date: " + str(datetime.datetime.now())


def get_os_string():
    """ Return a string with datetime """

    string = "OS: " + str(platform.system()) + " version "
    string += str(platform.release())
    return string


def get_git_hash():
    """ Return the current git hash """

    repo = git.Repo(search_parent_directories=True)
    return repo.head.object.hexsha


def get_git_string():
    """ Return a string with information on the git version """

    sha = get_git_hash()
    return "Git branch hash: " + sha


def get_python_string():
    """ Return a string with python information """

    return "Python version: " + str(platform.python_version())


def get_machine_string():
    """ Return a string with machine and MPI information """

    comm = mpi.COMM_WORLD
    rank = comm.Get_rank()
    size = comm.Get_size()
    machine = platform.node()
    string = "Machine: " + machine + " (" + str(rank+1) + "/" + str(size) + ")"

    return string


def get_gpu_string():
    """ Return a string with all MPI information """

    platforms = pyopencl.get_platforms()
    string = ""
    for p in platforms:
        string = "OpenCL: " + p.name + " (" + p.version + ") with devices: "
        devices = p.get_devices()
        for d in devices:
            string += d.name + "  "
    return string


def get_config_string(config):
    """ Return a string with the config file """

    string = "Config:\n\n"
    for sec in config.sections():
        string += "\n[" + sec + "]"
        for (key, val) in config.items(sec):
            string += "\n" + key + "=" + val
    return string + "\n\n"



if __name__ == "__main__":

    f = "test.csv"
    d = [{"a": 11, "b": 12, "e": 15}, {"a": 21, "c": 23, "b": 22}, 
         {"a": 31, "c": 33}, {"a": 41, "b": 42, "d": 44}, 
         {"a": 51, "b": 52, "c": 53, "d": 54}, {"a": 61, "c": 63, "d": 64}, 
         {"b": 72, "c": 73, "b": 72, "d": 74}]

    for i, e in enumerate(d):
        t_i = time.time()
        save_csv_row(e, f, i)
        print "Iteration " + str(i) + " : " + str(time.time() - t_i) + "s"