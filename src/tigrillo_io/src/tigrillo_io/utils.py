
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
import tigrillo
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
RESULTS_FOLDER = DATA_FOLDER + "/results"
CONFIG_FOLDER = DATA_FOLDER + "/configs"
I2C_CALIB_FILE = CONFIG_FOLDER + "/calibration.json"


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

def save_csv_row(dictionary, path, index):
    """ Save a dictionnary into one row of a csv file """

    with open(path, 'a') as f:
        w = csv.DictWriter(f, sorted(dictionary.keys()))
        if index == 0:
            w.writeheader()
        w.writerow(dictionary)


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
