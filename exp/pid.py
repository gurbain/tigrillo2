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

SIM_FILE = '/home/gabs48/src/quadruped/tigrillo2/exp/tigrillo.world'
MODEL_FILE = '/home/gabs48/.gazebo/models/tigrillo/model.sdf'
SAVE_FOLDER = '/home/gabs48/src/quadruped/tigrillo2/data/pid/results'
COMMAND = 80

class Score(object):

    def __init__(self):

        self.explosion_penalty = 10000
        self.fall_penalty = 0

        return      

    def cleanup(self):

        return

    def detect_failure(self):

        x_angle = np.array([i["ori_x"]/i["ori_w"] for i in self.imu_sig])

        # Fall
        if True in (x_angle < -0.4):
            sys.stdout.write("[Fall     ]\t")
            return self.fall_penalty

        # Explosion
        if ((x_angle == 0.0).sum() / float(x_angle.size)) > 0.1:
            sys.stdout.write("[Explosion]\t")
            return self.explosion_penalty

        sys.stdout.write("[Success  ]\t")
        return 0

    def get_score(self):

        # Detect a simulation failure (explosion, fall)
        penalty = self.detect_failure()
        if penalty == self.explosion_penalty:
            return penalty

        # Otherwise return the score
        if self.score_method  == "err_static":
            return self.score_static_err() + penalty

    def score_static_err(self):

        # Average the last 30 percent
        l = int(0.3 * len(self.mot_sig))
        fl_err = np.abs(np.mean(np.array([a["FL"] for a in self.mot_sig[-l:]]) - COMMAND))
        fr_err = np.abs(np.mean(np.array([a["FR"] for a in self.mot_sig[-l:]]) - COMMAND))
        bl_err = np.abs(np.mean(np.array([a["BL"] for a in self.mot_sig[-l:]]) - COMMAND))
        br_err = np.abs(np.mean(np.array([a["BR"] for a in self.mot_sig[-l:]]) - COMMAND))

        return fl_err + br_err + bl_err + br_err


class Optimization(Score):

    def __init__(self, model_file=MODEL_FILE, sim_file=SIM_FILE, save_folder=SAVE_FOLDER):

        self.sim_file = sim_file
        self.model_file = model_file
        self.save_folder = save_folder + "/" + utils.timestamp() + "/"
        utils.mkdir(self.save_folder)
        self.cma_data_filename = self.save_folder + "cmaes_evolution.pkl"

        self.model_conf = None

        self.sens_sig = []
        self.mot_sig = []
        self.imu_sig = []
        self.pose = []
        self.norm_params = []

        # Optimization metaparameter
        self.params_names   = ["Kp", "Ki", "Kd"]
        self.params_units   = ["", "", ""]
        self.params_unormed = []
        self.params_normed  = [0.5, 0.5, 0.5]
        self.params_min     = [0.1, 0.00001, 0.001]
        self.params_max     = [100, 0.01, 1]

        self.sim_time = 0
        self.stop_time = 10
        self.sim_timeout = 20
        self.pool_number = 1
        self.max_iter = 2000
        self.init_var = 0.2
        self.min = 0
        self.max = 1
        self.pop_size = 0
        self.score_method = "err_static"

        self.pool = 0
        self.it = 0
        self.score = 0
        self.t_init = None
        self.t_it_stop = None

        super(Optimization, self).__init__()

    def model(self):

        self.params_unormed = utils.unorm(self.params_normed, self.params_min, self.params_max)

        self.model_conf = model.model_config
        self.model_conf["p"] = self.params_unormed[0]
        self.model_conf["i"] = self.params_unormed[1]
        self.model_conf["d"] = self.params_unormed[2]

        fg = model.SDFileGenerator(self.model_conf, self.model_file, model_scale=1, gazebo=True)
        fg.generate()

    def act(self, t):

        return [COMMAND, COMMAND, COMMAND, COMMAND]

    def sim(self):

        # Create the simulation handle
        p = physics.Gazebo()
        p.start()

        # Wait for the gzserver process to be started
        t_init = time.time()
        while not p.is_sim_started():
            time.sleep(0.001)
            if (time.time() - t_init) > self.sim_timeout:
                p.stop()
                return -1

        # Perform simulation loop
        i = 0
        t = 0
        t_init = time.time()
        prev_t = -1
        while t < self.stop_time:

            # Timeout if failure
            if (time.time() - t_init) > self.sim_timeout:
                p.stop()
                return -1

            # Get sim time
            t = p.get_gazebo_time()

            # Actuate
            command = self.act(t)
            p.actuate(command)
            
            # Record simulation sensor signal
            self.sens_sig.append(p.get_sensors().copy())
            self.mot_sig.append(p.get_motors().copy())
            self.imu_sig.append(p.get_imu().copy())
            self.pose.append(p.get_pose().copy())

            # Wait here not to overload the sensor vector
            time.sleep(0.001)
            prev_t = t

        p.stop()
        return 0

    def cleanup(self):

        self.sens_sig = []
        self.mot_sig = []
        self.imu_sig = []
        self.pose = []
        self.model_conf = []
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
        res = self.sim()
        self.t_it_stop = time.time()
        self.pool += 1

        # Compare similarity and save
        if res == -1:
            sys.stdout.write("[Sim Crash]")
            score = np.nan
        else:
            score = self.get_score()
            sys.stdout.write("Score = {0:.4f}".format(score))
        sys.stdout.write("\t(st: " + str(self.stop_time) + \
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
            self.score = np.nanmean(np.array(score_arr))
            print("Iteration average score = {0:.4f}\n".format(self.score))
        else:
            self.score = self.pool_eval(parameters)
        
        # Save results and cleanup this class and its parent
        self.save()
        self.cleanup()
        self.it += 1

        return self.score

    def save(self):

        to_save = {"iter": self.it, "score": self.score, "params": self.params_normed, "mot_sig": self.mot_sig, \
                   "model_conf": self.model_conf, "elapsed time": self.t_it_stop - self.t_init}
        if self.it == 0:
            to_save["file_script"] = open(os.path.basename(__file__), 'r').read()
            to_save["sim_file"] = self.sim_file
            to_save["model_file"] = self.model_file
            to_save["sim_time"] = self.sim_time
            to_save["sim_timeout"] = self.sim_timeout
            to_save["stop_time"] = self.stop_time
            to_save["pop"] = self.pop_size
            to_save["pool_number"] = self.pool_number
            to_save["min"] = self.min
            to_save["max"] = self.max
            to_save["params_min"] = self.params_min
            to_save["params_max"] = self.params_max
            to_save["params_names"] = self.params_names
            to_save["params_units"] = self.params_units
            to_save["max_iter"] = self.max_iter
            to_save["init_var"] = self.init_var
            to_save["score_method"] = self.score_method

        utils.save_on_top(to_save, self.cma_data_filename)

    def run(self):

        # Get the recorded bag and init ROS
        ros.init_node('gait_optim', anonymous=True)
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