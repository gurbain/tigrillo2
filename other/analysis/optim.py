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

import cma
import numpy as np
from collections import OrderedDict
import os
import rosbag
import rospy as ros
from scipy.interpolate import interp1d
from scipy.spatial import distance
from scipy import signal
import sys
import time
       
import model
import physics
import utils

BAG_FILE = '/home/gabs48/src/quadruped/tigrillo2/data/analysis/bags/robot_model_calibration_data.bag'
MODEL_FILE = '/home/gabs48/.gazebo/models/tigrillo/model.sdf'
SIM_FILE = '/home/gabs48/src/quadruped/tigrillo2/other/analysis/tigrillo.world'
SAVE_FOLDER = '/home/gabs48/src/quadruped/tigrillo2/data/analysis/results'
TOPICS = ['/tigrillo_rob/uart_actuators', '/tigrillo_rob/uart_sensors', '/tigrillo_rob/i2c_sensors']


class Score(object):

    def __init__(self):

        self.start_eval_time = 21 # 10
        self.stop_eval_time = 44 # 47
        self.eval_points = 1500

        self.t_new = None
        self.zeros = None
        self.fl_rob_new = None
        self.fl_sim_new = None
        self.fr_rob_new = None
        self.fr_sim_new = None
        self.bl_rob_new = None
        self.bl_sim_new = None
        self.br_rob_new = None
        self.br_sim_new = None

        self.f_fl_sim = None
        self.f_fl_rob = None
        self.f_fr_sim = None
        self.f_fr_rob = None
        self.f_bl_sim = None
        self.f_bl_rob = None
        self.f_br_sim = None
        self.f_br_rob = None

        self.freq_fl_s = None
        self.t_fl_s = None
        self.s_fl = None
        self.freq_fr_s = None
        self.t_fr_s = None
        self.s_fr = None
        self.freq_bl_s = None
        self.t_bl_s = None
        self.s_bl = None
        self.freq_br_s = None
        self.t_br_s = None
        self.s_br = None

    def cleanup(self):

        self.t_new = None
        self.zeros = None
        self.fl_rob_new = None
        self.fl_sim_new = None
        self.fr_rob_new = None
        self.fr_sim_new = None
        self.bl_rob_new = None
        self.bl_sim_new = None
        self.br_rob_new = None
        self.br_sim_new = None
        self.fl_act_new = None
        self.f_fl_sim = None
        self.f_fl_rob = None
        self.f_fr_sim = None
        self.f_fr_rob = None
        self.f_bl_sim = None
        self.f_bl_rob = None
        self.f_br_sim = None
        self.f_br_rob = None
        self.freq_fl_s = None
        self.t_fl_s = None
        self.s_fl = None
        self.freq_fr_s = None
        self.t_fr_s = None
        self.s_fr = None
        self.freq_bl_s = None
        self.t_bl_s = None
        self.s_bl = None
        self.freq_br_s = None
        self.t_br_s = None
        self.s_br = None

    def get_score(self):

        if self.score_method  == "nrmse":
            return self.score_nrmse()
        elif self.score_method  == "spectrogram":
            return self.score_specto()
        elif self.score_method  == "av_period":
            return self.score_av_period()

    def score_av_period(self):

        self.interpolate()
        self.zeros = utils.zero_crossing(self.fl_act_new, self.t_new)
        #self.plot_av_period_before()
        c = len(self.zeros)
        r = int(len(self.t_new)/c)

        self.fl_rob_new = np.zeros([r, (c-1)])
        self.fl_sim_new = np.zeros([r, (c-1)])
        self.fr_rob_new = np.zeros([r, (c-1)])
        self.fr_sim_new = np.zeros([r, (c-1)])
        self.bl_rob_new = np.zeros([r, (c-1)])
        self.bl_sim_new = np.zeros([r, (c-1)])
        self.br_rob_new = np.zeros([r, (c-1)])
        self.br_sim_new = np.zeros([r, (c-1)])
        self.t_new = self.t_new[0:r]

        for i in range(c-1):
            x_new = np.linspace(self.zeros[i], self.zeros[i+1], r)
            self.fl_rob_new[:, i] = self.f_fl_rob(x_new)
            self.fl_sim_new[:, i] = self.f_fl_sim(x_new)
            self.fr_rob_new[:, i] = self.f_fr_rob(x_new)
            self.fr_sim_new[:, i] = self.f_fr_sim(x_new)
            self.bl_rob_new[:, i] = self.f_bl_rob(x_new)
            self.bl_sim_new[:, i] = self.f_bl_sim(x_new)
            self.br_rob_new[:, i] = self.f_br_rob(x_new)
            self.br_sim_new[:, i] = self.f_br_sim(x_new)

        # Average all periods and remove bias to center around 0
        fl_rob_mean, fl_sim_mean = utils.center_norm_2(np.mean(self.fl_rob_new, axis=1), np.mean(self.fl_sim_new, axis=1))
        fr_rob_mean, fr_sim_mean = utils.center_norm_2(np.mean(self.fr_rob_new, axis=1), np.mean(self.fr_sim_new, axis=1))
        bl_rob_mean, bl_sim_mean = utils.center_norm_2(np.mean(self.bl_rob_new, axis=1), np.mean(self.bl_sim_new, axis=1))
        br_rob_mean, br_sim_mean = utils.center_norm_2(np.mean(self.br_rob_new, axis=1), np.mean(self.br_sim_new, axis=1))
        #fr_rob_mean = np.mean(self.fr_rob_new, axis=1) - np.mean(np.mean(self.fr_rob_new, axis=1))
        #bl_rob_mean = np.mean(self.bl_rob_new, axis=1) - np.mean(np.mean(self.bl_rob_new, axis=1))
        #br_rob_mean = np.mean(self.br_rob_new, axis=1) - np.mean(np.mean(self.br_rob_new, axis=1))
        #fl_sim_mean = np.mean(self.fl_sim_new, axis=1) - np.mean(np.mean(self.fl_rob_new, axis=1)) 
        #fr_sim_mean = np.mean(self.fr_sim_new, axis=1) - np.mean(np.mean(self.fr_rob_new, axis=1))
        #bl_sim_mean = np.mean(self.bl_sim_new, axis=1) - np.mean(np.mean(self.bl_rob_new, axis=1))
        #br_sim_mean = np.mean(self.br_sim_new, axis=1) - np.mean(np.mean(self.br_rob_new, axis=1))

        #self.plot_av_period_after()
        rob_new = np.vstack((fl_rob_mean, fr_rob_mean, bl_rob_mean, br_rob_mean))
        sim_new = np.vstack((fl_sim_mean, fr_sim_mean, bl_sim_mean, br_sim_mean))
        #rob_new -= np.mean(np.mean(rob_new, axis=0), axis=1)
        #sim_new -= np.mean(np.mean(sim_new, axis=0), axis=1)
        score = utils.nrmse(rob_new, sim_new)
        return score

    def score_specto(self):

        # Create new time axis and interpolate
        self.eval_points = 20000
        self.interpolate()

        # Compute spectrograms
        fs = float(self.eval_points) / (self.stop_eval_time - self.start_eval_time)
        self.freq_fl_s, self.t_fl_s, s_fl_s = signal.spectrogram(self.fl_sim_new, fs)
        freq_fl_r, t_fl_r, s_fl_r = signal.spectrogram(self.fl_rob_new, fs)
        self.s_fl = s_fl_s[0:10][:] - s_fl_r[0:10][:]
        self.freq_fr_s, self.t_fr_s, s_fr_s = signal.spectrogram(self.fr_sim_new, fs)
        freq_fr_r, t_fr_r, s_fr_r = signal.spectrogram(self.fr_rob_new, fs)
        self.s_fr = s_fr_s[0:10][:] - s_fr_r[0:10][:]
        self.freq_bl_s, self.t_bl_s, s_bl_s = signal.spectrogram(self.bl_sim_new, fs)
        freq_bl_r, t_bl_r, s_bl_r = signal.spectrogram(self.bl_rob_new, fs)
        self.s_bl = s_bl_s[0:10][:] - s_bl_r[0:10][:]
        self.freq_br_s, self.t_br_s, s_br_s = signal.spectrogram(self.br_sim_new, fs)
        freq_br_r, t_br_r, s_br_r = signal.spectrogram(self.br_rob_new, fs)
        self.s_br = s_br_s[0:10][:] - s_br_r[0:10][:]

        #self.plot_specto()
        score = np.linalg.norm(self.s_fl) + np.linalg.norm(self.s_fr) + \
                np.linalg.norm(self.s_bl) + np.linalg.norm(self.s_br)
        return score

    def score_nrmse(self):
        
        self.eval_points = 1500
        self.interpolate()
        # self.plot_sensors()

        rob_new = np.vstack((self.fl_rob_new, self.fr_rob_new, self.bl_rob_new, self.br_rob_new))
        sim_new = np.vstack((self.fl_sim_new, self.fr_sim_new, self.bl_sim_new, self.br_sim_new))
        score = utils.nrmse(rob_new, sim_new)
        return score

    def interpolate(self):

        # Remove items taken at the same timestep
        sens_sim = utils.filter_duplicate(self.sens_sim_sig)
        sens_rob = utils.filter_duplicate(self.sens_rob_sig)
        act = utils.filter_duplicate(self.act_sig)

        # Get all data in a numpy array and interpolate functions
        t_sim = np.array([d["time"] for d in sens_sim])
        t_rob = np.array([d["time"] for d in sens_rob])
        fl_sim = np.array([d["FL"] for d in sens_sim])
        fl_rob = np.array([d["FL"] for d in sens_rob])
        fr_sim = np.array([d["FR"] for d in sens_sim])
        fr_rob = np.array([d["FR"] for d in sens_rob])
        bl_sim = np.array([d["BL"] for d in sens_sim])
        bl_rob = np.array([d["BL"] for d in sens_rob])
        br_sim = np.array([d["BR"] for d in sens_sim])
        br_rob = np.array([d["BR"] for d in sens_rob])
        self.f_fl_sim = interp1d(t_sim, fl_sim, kind='cubic', assume_sorted=False)
        self.f_fl_rob = interp1d(t_rob, fl_rob, kind='cubic', assume_sorted=False)
        self.f_fr_sim = interp1d(t_sim, fr_sim, kind='cubic', assume_sorted=False)
        self.f_fr_rob = interp1d(t_rob, fr_rob, kind='cubic', assume_sorted=False)
        self.f_bl_sim = interp1d(t_sim, bl_sim, kind='cubic', assume_sorted=False)
        self.f_bl_rob = interp1d(t_rob, bl_rob, kind='cubic', assume_sorted=False)
        self.f_br_sim = interp1d(t_sim, br_sim, kind='cubic', assume_sorted=False)
        self.f_br_rob = interp1d(t_rob, br_rob, kind='cubic', assume_sorted=False)

        # Create new time axis and interpolate
        self.t_new = np.linspace(self.start_eval_time, self.stop_eval_time, self.eval_points)
        self.fl_rob_new = self.f_fl_rob(self.t_new)
        self.fl_sim_new = self.f_fl_sim(self.t_new)
        self.fr_rob_new = self.f_fr_rob(self.t_new)
        self.fr_sim_new = self.f_fr_sim(self.t_new)
        self.bl_rob_new = self.f_bl_rob(self.t_new)
        self.bl_sim_new = self.f_bl_sim(self.t_new)
        self.br_rob_new = self.f_br_rob(self.t_new)
        self.br_sim_new = self.f_br_sim(self.t_new)
        self.fl_act_new = self.f_fl_act(self.t_new)

    def plot_sensors(self):

        plt.plot(self.t_new, self.fl_rob_new, linewidth=1, label="Front Left Robot")
        plt.plot(self.t_new, self.fl_sim_new, linewidth=1, label="Front Left Simulation")
        plt.savefig(self.save_folder + "front_left_" + str(self.it) + ".png", format='png', dpi=300)
        plt.close()
        plt.plot(self.t_new, self.fr_rob_new, linewidth=1, label="Front Right Robot")
        plt.plot(self.t_new, self.fr_sim_new, linewidth=1, label="Front Right Simulation")
        plt.savefig(self.save_folder + "front_right_" + str(self.it) + ".png", format='png', dpi=300)
        plt.close()
        plt.plot(self.t_new, self.bl_rob_new, linewidth=1, label="Back Left Robot")
        plt.plot(self.t_new, self.bl_sim_new, linewidth=1, label="Back Left Simulation")
        plt.savefig(self.save_folder + "back_left_" + str(self.it) + ".png", format='png', dpi=300)
        plt.close()
        plt.plot(self.t_new, self.br_rob_new, linewidth=1, label="Back Right Robot")
        plt.plot(self.t_new, self.br_sim_new, linewidth=1, label="Back Right Simulation")
        plt.savefig(self.save_folder + "back_right_" + str(self.it) + ".png", format='png', dpi=300)
        plt.close()

    def plot_specto(self):

        plt.pcolormesh(self.t_fl_s, self.freq_fl_s[0:10], self.s_fl)
        plt.ylabel('Frequency [Hz]')
        plt.xlabel('Time [sec]')
        plt.savefig(self.save_folder + "specto_diff_fl_" + str(self.it) + ".png", format='png', dpi=300)
        plt.close()
        plt.pcolormesh(self.t_fr_s, self.freq_fr_s[0:10], self.s_fr)
        plt.ylabel('Frequency [Hz]')
        plt.xlabel('Time [sec]')
        plt.savefig(self.save_folder + "specto_diff_fr_" + str(self.it) + ".png", format='png', dpi=300)
        plt.close()
        plt.pcolormesh(self.t_bl_s, self.freq_bl_s[0:10], self.s_bl)
        plt.ylabel('Frequency [Hz]')
        plt.xlabel('Time [sec]')
        plt.savefig(self.save_folder + "specto_diff_bl_" + str(self.it) + ".png", format='png', dpi=300)
        plt.close()
        plt.pcolormesh(self.t_br_s, self.freq_br_s[0:10], self.s_br)
        plt.ylabel('Frequency [Hz]')
        plt.xlabel('Time [sec]')
        plt.savefig(self.save_folder + "specto_diff_br_" + str(self.it) + ".png", format='png', dpi=300)
        plt.close()
    
    def plot_av_period_before(self):

        act_new = self.fl_act_new * 127.68310365946067
        act_new += 82.5

        mi = min(act_new)
        ma = max(act_new)
        plt.plot(self.t_new, act_new, linewidth=1, label="FL Act")
        plt.plot(self.t_new, self.fl_rob_new, linewidth=1, label="FL Rob")
        for t in self.zeros:
             plt.plot([t, t], [mi, ma], linewidth=1)
        plt.savefig(self.save_folder + "period_before_" + str(self.it) + ".png", format='png', dpi=300)
        plt.close()

    def plot_av_period_after(self):

        plt.plot(self.t_new, self.fl_rob_new, linewidth=0.4, color="skyblue", label="Superposed FL Rob")
        plt.plot(self.t_new, np.mean(self.fl_rob_new, axis=1), linewidth=2, color="b", label="Averaged FL Rob")
        plt.plot(self.t_new, self.fl_sim_new, linewidth=0.4, color="orchid", label="Superposed FL Sim")
        plt.plot(self.t_new, np.mean(self.fl_sim_new, axis=1), linewidth=2, color="r", label="Averaged FL Sim")
        plt.xlim([self.t_new[0], self.t_new[-1]])
        plt.savefig(self.save_folder + "period_after_FL_" + str(self.it) + ".png", format='png', dpi=300)
        plt.close()
        plt.plot(self.t_new, self.fr_rob_new, linewidth=0.4, color="skyblue", label="Superposed FR Rob")
        plt.plot(self.t_new, np.mean(self.fr_rob_new, axis=1), linewidth=2, color="b", label="Averaged FR Rob")
        plt.plot(self.t_new, self.fr_sim_new, linewidth=0.4, color="orchid", label="Superposed FR Sim")
        plt.plot(self.t_new, np.mean(self.fr_sim_new, axis=1), linewidth=2, color="r", label="Averaged FR Sim")
        plt.xlim([self.t_new[0], self.t_new[-1]])
        plt.savefig(self.save_folder + "period_after_FR_" + str(self.it) + ".png", format='png', dpi=300)
        plt.close()
        plt.plot(self.t_new, self.bl_rob_new, linewidth=0.4, color="skyblue", label="Superposed BL Rob")
        plt.plot(self.t_new, np.mean(self.bl_rob_new, axis=1), linewidth=2, color="b", label="Averaged BL Rob")
        plt.plot(self.t_new, self.bl_sim_new, linewidth=0.4, color="orchid", label="Superposed BL Sim")
        plt.plot(self.t_new, np.mean(self.bl_sim_new, axis=1), linewidth=2, color="r", label="Averaged BL Sim")
        plt.xlim([self.t_new[0], self.t_new[-1]])
        plt.savefig(self.save_folder + "period_after_BL_" + str(self.it) + ".png", format='png', dpi=300)
        plt.close()
        plt.plot(self.t_new, self.br_rob_new, linewidth=0.4, color="skyblue", label="Superposed BR Rob")
        plt.plot(self.t_new, np.mean(self.br_rob_new, axis=1), linewidth=2, color="b", label="Averaged BR Rob")
        plt.plot(self.t_new, self.br_sim_new, linewidth=0.4, color="orchid", label="Superposed BR Sim")
        plt.plot(self.t_new, np.mean(self.br_sim_new, axis=1), linewidth=2, color="r", label="Averaged BR Sim")
        plt.xlim([self.t_new[0], self.t_new[-1]])
        plt.savefig(self.save_folder + "period_after_BR_" + str(self.it) + ".png", format='png', dpi=300)
        plt.close()


class Optimization(Score):

    def __init__(self, bag_file=BAG_FILE, model_file=MODEL_FILE, sim_file=SIM_FILE, topics=TOPICS, save_folder=SAVE_FOLDER):

        self.bag_file = bag_file
        self.topic_list = topics
        self.sim_file = sim_file
        self.model_file = model_file
        self.save_folder = save_folder + "/" + utils.timestamp() + "/"

        self.act_sig = []
        self.act_t = []
        self.sens_sim_sig = []
        self.sens_rob_sig = []
        self.imu_sim_sig = []
        self.imu_rob_sig = []
        self.norm_params = []

        # Optimization metaparameter
        self.params_names = ["Front Mass", "Back Mass", "Front Friction mu1", "Front Friction mu2", "Front Friction Contact Depth",
                             "Back Friction mu1", "Back Friction mu2", "Back Friction Contact Depth", "Front Damping",
                             "Front Stiffness", "Back Damping", "Back Stiffness", "Motor Correction Factor",
                             "Front Motor Offset", "Back Motor Offset"]
        self.params_units = ["kg", "kg", " ", " ", "mm", " ", " ", "mm", "N.s/m", "N/m", "N.s/m", "N/m", " ", "Degrees", " ", "Degrees"]
        self.params_normed = [0.5, 0.5, 0.8, 0.8, 0.5, 0.8, 0.8, 0.5, 0.1, 0.1, 0.1, 0.1, 0.5, 0.5, 0.5]
        self.params_unormed = []
        self.params_min = [0.2, 0.2, 0.1, 0.1, 0.01, 0.1, 0.1, 0.01, 0.001, 0.01, 0.001, 0.01, 0.5, -math.pi/6, -math.pi/6]
        self.params_max = [0.5, 0.5, 20000, 20000, 0.1, 20000, 20000, 0.1, 0.5, 50, 0.5, 50, 1.5, math.pi/6, math.pi/6]

        self.sim_time = 0
        self.start_time = 20 # 5
        self.stop_time = 45 # 50
        self.pool_number = 1
        self.max_iter = 2000
        self.init_var = 0.2
        self.min = 0
        self.max = 1
        self.pop_size = 0
        self.score_method = "av_period"

        self.it = 0
        self.pool = 0
        self.score = 0
        self.t_init = None
        self.t_it_stop = None

        super(Optimization, self).__init__()

    def load(self):

        try:
            with rosbag.Bag(self.bag_file, 'r') as bag:
                t_init = bag.get_start_time()
                for topic, msg, t in bag.read_messages(topics=self.topic_list):

                    if self.start_time < float(t.to_time() - t_init) < self.stop_time:
                        if topic == '/tigrillo_rob/uart_actuators':
                            d = dict()
                            d["time"] = float(t.to_time() - t_init)
                            d["FL"] = float(msg.FL)
                            d["FR"] = float(msg.FR)
                            d["BL"] = float(msg.BL)
                            d["BR"] = float(msg.BR)
                            self.act_sig.append(d)
                        if topic == '/tigrillo_rob/uart_sensors':
                            d = dict()
                            d["time"] = float(t.to_time() - t_init)
                            d["FL"] = float(msg.FL)
                            d["FR"] = float(msg.FR)
                            d["BL"] = float(msg.BL)
                            d["BR"] = float(msg.BR)
                            self.sens_rob_sig.append(d)
                        if topic == '/tigrillo_rob/i2c_sensors':
                            d = dict()
                            d["time"] = float(t.to_time() - t_init)
                            d["acc_x"] = float(msg.acc_x)
                            d["acc_y"] = float(msg.acc_y)
                            d["acc_z"] = float(msg.acc_z)
                            d["grav_x"] = float(msg.grav_x)
                            d["grav_y"] = float(msg.grav_y)
                            d["grav_z"] = float(msg.grav_z)
                            d["h"] = float(msg.h)
                            d["r"] = float(msg.r)
                            d["p"] = float(msg.p)
                            self.imu_rob_sig.append(d)

                self.act_t = [d["time"] for d in self.act_sig]

        except rosbag.bag.ROSBagException as err:
            print("Error when loading the bag: " + str(e))
            return None

        utils.mkdir(self.save_folder)
        self.cma_data_filename = self.save_folder + "cmaes_evolution.pkl"

        # Add interpolation function for actuation signal
        self.t_act = np.array([d["time"] for d in self.act_sig])
        self.fl_act = np.array([d["FL"] for d in self.act_sig])
        self.fr_act = np.array([d["FR"] for d in self.act_sig])
        self.bl_act = np.array([d["BL"] for d in self.act_sig])
        self.br_act = np.array([d["BR"] for d in self.act_sig])
        self.f_fl_act = interp1d(self.t_act, self.fl_act, assume_sorted=False, fill_value="extrapolate")
        self.f_fr_act = interp1d(self.t_act, self.fr_act, assume_sorted=False, fill_value="extrapolate")
        self.f_bl_act = interp1d(self.t_act, self.bl_act, assume_sorted=False, fill_value="extrapolate")
        self.f_br_act = interp1d(self.t_act, self.br_act, assume_sorted=False, fill_value="extrapolate")

        return

    def model(self):

        # Create config
        self.params_unormed = utils.unorm(self.params_normed, self.params_min, self.params_max)
        self.conf = model.model_config
        self.conf["body"]["front"]["mass"] = self.params_unormed[0]
        self.conf["body"]["hind"]["mass"] = self.params_unormed[1]
        self.conf["legs"]["FL"]["foot"]["mu1"] = self.params_unormed[2]
        self.conf["legs"]["FL"]["foot"]["mu2"] = self.params_unormed[3]
        self.conf["legs"]["FL"]["foot"]["contact_depth"] = self.params_unormed[4]
        self.conf["legs"]["FR"]["foot"]["mu1"] = self.params_unormed[2]
        self.conf["legs"]["FR"]["foot"]["mu2"] = self.params_unormed[3]
        self.conf["legs"]["FR"]["foot"]["contact_depth"] = self.params_unormed[4]
        self.conf["legs"]["BL"]["foot"]["mu1"] = self.params_unormed[5]
        self.conf["legs"]["BL"]["foot"]["mu2"] = self.params_unormed[6]
        self.conf["legs"]["BL"]["foot"]["contact_depth"] = self.params_unormed[7]
        self.conf["legs"]["BR"]["foot"]["mu1"] = self.params_unormed[5]
        self.conf["legs"]["BR"]["foot"]["mu2"] = self.params_unormed[6]
        self.conf["legs"]["BR"]["foot"]["contact_depth"] = self.params_unormed[7]
        self.conf["legs"]["FL"]["knee_damping"] = self.params_unormed[8]
        self.conf["legs"]["FL"]["spring_stiffness"] = self.params_unormed[9]
        self.conf["legs"]["FR"]["knee_damping"] = self.params_unormed[8]
        self.conf["legs"]["FR"]["spring_stiffness"] = self.params_unormed[9]
        self.conf["legs"]["BL"]["knee_damping"] = self.params_unormed[10]
        self.conf["legs"]["BL"]["spring_stiffness"] = self.params_unormed[11]
        self.conf["legs"]["BR"]["knee_damping"] = self.params_unormed[10]
        self.conf["legs"]["BR"]["spring_stiffness"] = self.params_unormed[11]

        fg = model.SDFileGenerator(self.conf, self.model_file, model_scale=1, gazebo=True)
        fg.generate()

    def act(self, t):

        fl = self.f_fl_act(t) * self.params_unormed[11] + self.params_unormed[12]
        fr = self.f_fr_act(t) * self.params_unormed[11] + self.params_unormed[12]
        bl = self.f_bl_act(t) * self.params_unormed[11] + self.params_unormed[13]
        br = self.f_br_act(t) * self.params_unormed[11] + self.params_unormed[13]

        return [fl, fr, bl, br]

    def sim(self):

        # Create the simulation handle
        p = physics.Gazebo()
        p.start()


        # Wait for the gzserver process to be started
        while not p.is_sim_started():
            time.sleep(0.001)

        # Perform simulation loop
        t = self.start_time
        i = 0
        while t < self.stop_time:
            # Get sim time
            t = p.get_gazebo_time() + self.start_time

            # Actuate
            command = self.act(t)
            p.actuate(command)
            
            # Record simulation sensor signal
            s = p.get_sensors().copy()
            s["time"] += self.start_time
            self.sens_sim_sig.append(s)

            # Wait here not to overload the sensor vector
            time.sleep(0.005)

        p.stop()
        return 0

    def cleanup(self):

        self.sens_sim_sig = []
        self.imu_sim_sig = []
        self.conf = []
        self.norm_params = []
        super(Optimization, self).cleanup()

    def pool_eval(self, parameters):

        # Init
        t_it_init = time.time()
        sys.stdout.write("Iteration " + str(self.it+1) + ":\t")

        # Create the new Tigrillo model
        self.params_normed = parameters
        self.model()

        # Perform simulation
        self.sim()
        self.t_it_stop = time.time()
        self.pool += 1

        # Compare similarity and save
        score = self.get_score()
        sys.stdout.write("Score = {0:.4f}".format(score))
        sys.stdout.write("\t\t(st: " + str(self.stop_time - self.start_time) + \
                         "s, rt: {0:.2f}s)\n".format(self.t_it_stop - t_it_init))

        return score

    def eval(self, parameters):

        # Manage multiple pooled simulation to average
        if self.pool_number > 1:
            score_arr = []
            for n in range(self.pool_number):
                sys.stdout.write("Pool " + str(self.pool+1) + " | ")
                score_arr.append(self.pool_eval(parameters))
            
            self.pool = 0
            self.score = sum(score_arr) / len(score_arr)
            print("Iteration average score = {0:.4f}\n".format(self.score))
        else:
            self.score = self.pool_eval(parameters)
        
        # Save results and cleanup this class and its parent
        self.save()
        self.cleanup()
        self.it += 1

        return self.score

    def save(self):

        to_save = {"iter": self.it, "score": self.score, "params": self.params_normed, \
                   "config": self.conf, "elapsed time": self.t_it_stop - self.t_init, \
                   "fl_sim": self.fl_sim_new, "fr_sim": self.fr_sim_new, \
                   "bl_sim": self.bl_sim_new, "br_sim": self.br_sim_new}
        if self.it == 0:
            to_save["file_script"] = open(os.path.basename(__file__), 'r').read()
            to_save["sim_file"] = self.sim_file
            to_save["bag_file"] = self.bag_file
            to_save["model_file"] = self.model_file
            to_save["sim_time"] = self.sim_time
            to_save["start_time"] = self.start_time
            to_save["stop_time"] = self.stop_time
            to_save["pop"] = self.pop_size
            to_save["pool_number"] = self.pool_number
            to_save["min"] = self.min
            to_save["max"] = self.max
            to_save["params_min"] = self.params_min
            to_save["params_max"] = self.params_max
            to_save["params_names"] = self.params_names
            to_save["params_units"] = self.params_units
            to_save["eval_points"] = self.eval_points
            to_save["start_eval_time"] = self.start_eval_time
            to_save["stop_eval_time"] = self.stop_eval_time
            to_save["max_iter"] = self.max_iter
            to_save["init_var"] = self.init_var
            to_save["score_method"] = self.score_method
            to_save["fl_rob"] = self.fl_rob_new
            to_save["fr_rob"] = self.fr_rob_new
            to_save["bl_rob"] = self.bl_rob_new
            to_save["br_rob"] = self.br_rob_new
            to_save["t"] = self.t_new
            to_save["fl_act"] = self.fl_act
            to_save["fr_act"] = self.fr_act
            to_save["bl_act"] = self.bl_act
            to_save["br_act"] = self.br_act
            to_save["t_act"] = self.t_act

        utils.save_on_top(to_save, self.cma_data_filename)

    def run(self):

        # Get the recorded bag and init ROS
        self.load()
        ros.init_node('optim', anonymous=True)
        ros.on_shutdown(utils.cleanup)
    
        # Init algorithm
        es = cma.CMAEvolutionStrategy(self.params_normed, self.init_var,
                                      {'boundary_handling': 'BoundTransform ', 'bounds': [self.min, self.max],
                                       'maxfevals': self.max_iter, 'verbose': -9})
        self.pop_size = es.popsize
        self.t_init = time.time()

        # Run optimization
        print("== Start Optimization process with dim of " + str(len(self.params_normed)) + \
              " and population size of " + str(self.pop_size) + " ==\n")
        while not es.stop():
            solutions = es.ask()
            es.tell(solutions, [self.eval(l) for l in solutions])
        t_stop = time.time()
        res = es.result()

        # Return optimum
        opt_time = t_stop - self.t_init
        opt_param = res[0]
        opt_score = res[1]
        print("== Finish Optimization process with opt score = {0:.3f} and params = ".format(opt_score) + \
              str(opt_param) + " == ")
        print("== Optimization time for " + str(self.it) + " epochs: {0:.1f}s.".format(opt_time) + \
              " {0:.3f}s in average per iteration ==".format(opt_time/self.it))

        exit()


if __name__ == '__main__':

    o = Optimization()
    o.run()