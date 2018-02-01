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

from bisect import bisect
import cma
import numpy as np
from collections import OrderedDict
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

        self.start_eval_time = 25
        self.stop_eval_time = 35
        self.eval_points = 20000

        self.t_new = None
        self.fl_rob_new = None
        self.fl_sim_new = None
        self.fr_rob_new = None
        self.fr_sim_new = None
        self.bl_rob_new = None
        self.bl_sim_new = None
        self.br_rob_new = None
        self.br_sim_new = None

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
        self.fl_rob_new = None
        self.fl_sim_new = None
        self.fr_rob_new = None
        self.fr_sim_new = None
        self.bl_rob_new = None
        self.bl_sim_new = None
        self.br_rob_new = None
        self.br_sim_new = None
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
        
        #self.eval_points = 1500
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

        # Get all data in a numpy array and interpolate functions
        t_sim = np.array([d["time"]for d in sens_sim])
        t_rob = np.array([d["time"]for d in sens_rob])
        fl_sim = np.array([d["FL"]for d in sens_sim])
        fl_rob = np.array([d["FL"]for d in sens_rob])
        fr_sim = np.array([d["FR"]for d in sens_sim])
        fr_rob = np.array([d["FR"]for d in sens_rob])
        bl_sim = np.array([d["BL"]for d in sens_sim])
        bl_rob = np.array([d["BL"]for d in sens_rob])
        br_sim = np.array([d["BR"]for d in sens_sim])
        br_rob = np.array([d["BR"]for d in sens_rob])
        f_fl_sim = interp1d(t_sim, fl_sim, kind='cubic', assume_sorted=False)
        f_fl_rob  = interp1d(t_rob, fl_rob, kind='cubic', assume_sorted=False)
        f_fr_sim = interp1d(t_sim, fr_sim, kind='cubic', assume_sorted=False)
        f_fr_rob  = interp1d(t_rob, fr_rob, kind='cubic', assume_sorted=False)
        f_bl_sim = interp1d(t_sim, bl_sim, kind='cubic', assume_sorted=False)
        f_bl_rob  = interp1d(t_rob, bl_rob, kind='cubic', assume_sorted=False)
        f_br_sim = interp1d(t_sim, br_sim, kind='cubic', assume_sorted=False)
        f_br_rob  = interp1d(t_rob, br_rob, kind='cubic', assume_sorted=False)

        # Create new time axis and interpolate
        self.t_new = np.linspace(self.start_eval_time, self.stop_eval_time, self.eval_points)
        self.fl_rob_new = f_fl_rob(self.t_new)
        self.fl_sim_new = f_fl_sim(self.t_new)
        self.fr_rob_new = f_fr_rob(self.t_new)
        self.fr_sim_new = f_fr_sim(self.t_new)
        self.bl_rob_new = f_bl_rob(self.t_new)
        self.bl_sim_new = f_bl_sim(self.t_new)
        self.br_rob_new = f_br_rob(self.t_new)
        self.br_sim_new = f_br_sim(self.t_new)

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
        self.params = [0.5, 0.8, 0.8, 0.5, 0.8, 0.8, 0.5, 0.1, 0.1, 0.1, 0.1]
        self.sim_time = 0
        self.start_time = 20
        self.stop_time = 40
        self.pool_number = 1
        self.max_iter = 300
        self.init_var = 0.2
        self.min = 0
        self.max = 1
        self.score_method = "nrmse"

        self.it = 0
        self.pool = 0
        self.score = 0

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

        return

    def unorm(self):

        p = self.params
        np = self.norm_params

        self.total_mass = 0.5
        self.min_perc_mass = 0.1
        self.max_perc_mass = 0.9
        self.min_mu = 0.1
        self.max_mu = 20000
        self.min_cd = 0.01
        self.max_cd = 0.1
        self.min_damping = 0.001
        self.max_damping = 0.5
        self.min_spring = 0.01
        self.max_spring = 50

        p[0] = (self.min_perc_mass + np[0] * (self.max_perc_mass - self.min_perc_mass)) * self.total_mass
        p[1] = self.min_mu + np[1] * (self.max_mu - self.min_mu)
        p[2] = self.min_mu + np[2] * (self.max_mu - self.min_mu)
        p[3] = self.min_cd + np[3] * (self.max_cd - self.min_cd)
        p[4] = self.min_mu + np[4] * (self.max_mu - self.min_mu)
        p[5] = self.min_mu + np[5] * (self.max_mu - self.min_mu)
        p[6] = self.min_cd + np[6] * (self.max_cd - self.min_cd)
        p[7] = self.min_damping + np[7] * (self.max_damping - self.min_damping)
        p[8] = self.min_spring + np[8] * (self.max_spring - self.min_spring)
        p[9] = self.min_damping + np[9] * (self.max_damping - self.min_damping)
        p[10] = self.min_spring + np[10] * (self.max_spring - self.min_spring)
        self.params = p

    def model(self):

        # Create config
        self.unorm()
        self.conf = model.model_config
        self.conf["body"]["front"]["mass"] = self.params[0]
        self.conf["body"]["hind"]["mass"] = self.total_mass - self.params[0]
        self.conf["legs"]["FL"]["foot"]["mu1"] = self.params[1]
        self.conf["legs"]["FL"]["foot"]["mu2"] = self.params[2]
        self.conf["legs"]["FL"]["foot"]["contact_depth"] = self.params[3]
        self.conf["legs"]["FR"]["foot"]["mu1"] = self.params[1]
        self.conf["legs"]["FR"]["foot"]["mu2"] = self.params[2]
        self.conf["legs"]["FR"]["foot"]["contact_depth"] = self.params[3]
        self.conf["legs"]["BL"]["foot"]["mu1"] = self.params[4]
        self.conf["legs"]["BL"]["foot"]["mu2"] = self.params[5]
        self.conf["legs"]["BL"]["foot"]["contact_depth"] = self.params[6]
        self.conf["legs"]["BR"]["foot"]["mu1"] = self.params[4]
        self.conf["legs"]["BR"]["foot"]["mu2"] = self.params[5]
        self.conf["legs"]["BR"]["foot"]["contact_depth"] = self.params[6]
        self.conf["legs"]["FL"]["knee_damping"] = self.params[7]
        self.conf["legs"]["FL"]["spring_stiffness"] = self.params[8]
        self.conf["legs"]["FR"]["knee_damping"] = self.params[7]
        self.conf["legs"]["FR"]["spring_stiffness"] = self.params[8]
        self.conf["legs"]["BL"]["knee_damping"] = self.params[9]
        self.conf["legs"]["BL"]["spring_stiffness"] = self.params[10]
        self.conf["legs"]["BR"]["knee_damping"] = self.params[9]
        self.conf["legs"]["BR"]["spring_stiffness"] = self.params[10]

        fg = model.SDFileGenerator(self.conf, self.model_file, model_scale=1, gazebo=True)
        fg.generate()

    def act(self, t):

        ind = bisect(self.act_t, t)
        if ind >= len(self.act_t):
            return [self.act_sig[-1]["FL"], self.act_sig[-1]["FR"], 
                    self.act_sig[-1]["BL"], self.act_sig[-1]["BR"]]
        t1 = self.act_sig[ind-1]["time"]
        t2 = self.act_sig[ind]["time"]
        fl1 = self.act_sig[ind-1]["FL"]
        fl2 = self.act_sig[ind]["FL"]
        fl = fl1 + ((fl2- fl1) * (t - t1) / (t2 - t1))
        fr1 = self.act_sig[ind-1]["FR"]
        fr2 = self.act_sig[ind]["FR"]
        fr = fr1 + ((fr2- fr1) * (t - t1) / (t2 - t1))
        bl1 = self.act_sig[ind-1]["BL"]
        bl2 = self.act_sig[ind]["BL"]
        bl = bl1 + ((bl2- bl1) * (t - t1) / (t2 - t1))
        br1 = self.act_sig[ind-1]["BR"]
        br2 = self.act_sig[ind]["BR"]
        br = br1 + ((br2- br1) * (t - t1) / (t2 - t1))

        #print(fl1, fl, fl2)
        #print(t1, t, t2)
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
        return

    def cleanup(self):

        self.sens_sim_sig = []
        self.imu_sim_sig = []
        self.conf = []
        self.norm_params = []
        super(Optimization, self).cleanup()

    def pool_eval(self, parameters):

        # Init
        t_init = time.time()
        sys.stdout.write("Iteration " + str(self.it+1) + ":\t")

        # Create the new Tigrillo model
        self.norm_params = parameters
        self.model()

        # Perform simulation
        self.sim()
        self.pool += 1

        # Compare similarity and save
        score = self.get_score()
        sys.stdout.write("Score = {0:.4f}".format(score))
        sys.stdout.write("\t\t(st: " + str(self.stop_time - self.start_time) + \
                         "s, rt: {0:.2f}s)\n".format(time.time() - t_init))

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

        to_save = {"iter": self.it, "score": self.score, "sim_time": self.sim_time, \
                   "sim_time": self.sim_time, "start_time": self.start_time, \
                   "stop_time": self.stop_time, "pool_number": self.pool_number, \
                   "min": self.min,  "max": self.max,  "score_method": self.score_method, \
                   "params": self.params, "config": self.conf, "eval_points": self.eval_points, \
                   "start_eval_time": self.start_eval_time, "stop_eval_points": self.stop_eval_time, \
                   "max_iter": self.max_iter, "init_var": self.init_var, "t": self.t_new, \
                   "fl_rob": self.fl_rob_new, "fl_sim": self.fl_sim_new, \
                   "fr_rob": self.fr_rob_new, "fr_sim": self.fr_sim_new, \
                   "bl_rob": self.bl_rob_new, "bl_sim": self.bl_sim_new, \
                   "br_rob": self.br_rob_new, "br_sim": self.br_sim_new, \
                   "bag_file": self.bag_file, "sim_file": self.sim_file}

        utils.save_on_top(to_save, self.cma_data_filename)

    def run(self):

        # Get the recorded bag and init ROS
        self.load()
        ros.init_node('optim', anonymous=True)
        ros.on_shutdown(utils.cleanup)
    
        # Init algorithm
        es = cma.CMAEvolutionStrategy(self.params, self.init_var,
                                      {'boundary_handling': 'BoundTransform ', 'bounds': [self.min, self.max],
                                       'maxfevals': self.max_iter, 'verbose': -9})
        pop_size = es.popsize
        t_init = time.time()

        # Run optimization
        print("== Start Optimization process with dim of " + str(len(self.params)) + \
              " and population size of " + str(pop_size) + " ==\n")
        while not es.stop():
            solutions = es.ask()
            es.tell(solutions, [self.eval(l) for l in solutions])
        t_stop = time.time()
        res = es.result()

        # Return optimum
        opt_time = t_stop - t_init
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