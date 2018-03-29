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

from tigrillo_ctrl import control

SIM_FILE = '/home/gabs48/src/quadruped/tigrillo2/other/analysis/tigrillo.world'
MODEL_FILE = '/home/gabs48/.gazebo/models/tigrillo/model.sdf'
SAVE_FOLDER = '/home/gabs48/src/quadruped/tigrillo2/data/gait/results'


class Score(object):

    def __init__(self):

        return      

    def cleanup(self):

        return

    def get_score(self):

        if self.score_method  == "distance":
            return self.score_dist()

    def score_dist(self):

        return  -math.sqrt(self.pose[-1]["x"]** 2 + self.pose[-1]["y"] ** 2)


class Optimization(Score):

    def __init__(self, model_file=MODEL_FILE, sim_file=SIM_FILE, save_folder=SAVE_FOLDER):

        self.sim_file = sim_file
        self.model_file = model_file
        self.save_folder = save_folder + "/" + utils.timestamp() + "/"
        utils.mkdir(self.save_folder)
        self.cma_data_filename = self.save_folder + "cmaes_evolution.pkl"

        self.model_conf = None
        self.cpg_conf = None
        self.cpg = None

        self.sens_sig = []
        self.imu_sig = []
        self.pose = []
        self.norm_params = []

        # Optimization metaparameter
        self.params_names   = ["Mu Back", "Mu Front", "Duty Factor Back", "Duty Factor Front", "Phase Offset Back", 
                               "Phase Offset Front", " Offset Back", "Offset Front", "Omega"]
        self.params_units   = ["Degrees", "Degrees", "Ratio", "Ratio", "Radians", "Radians", "Degrees", "Degrees", "Radians/s"]
        self.params_unormed = []
        self.params_normed  = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
        self.params_min     = [0, 0, 0, 0, 0, 0, -60, -60, 0]
        self.params_max     = [100, 100, 1, 1, math.pi, math.pi, 60, 60, 4 * math.pi]

        self.sim_time = 0
        self.stop_time = 10
        self.sim_timeout = 60
        self.pool_number = 3
        self.max_iter = 5000
        self.init_var = 0.4
        self.min = 0
        self.max = 1
        self.pop_size = 0
        self.score_method = "distance"

        self.pool = 0
        self.it = 0
        self.score = 0
        self.t_init = None
        self.t_it_stop = None

        super(Optimization, self).__init__()

    def model(self):

        # Commented: We don't change the model parameters right now!!
        # self.model_conf = model.model_config
        # self.model_conf["body"]["front"]["mass"] = 1.0
        # fg = model.SDFileGenerator(self.model_conf, self.model_file, model_scale=1, gazebo=True)
        # fg.generate()

        # We set the controller here
        self.params_unormed = utils.unorm(self.params_normed, self.params_min, self.params_max)
        dir(control)
        self.cpg_conf = control.cpg_config
        mu_b   = self.params_unormed[0]
        mu_f   = self.params_unormed[1]
        duty_b = self.params_unormed[2]
        duty_f = self.params_unormed[3]
        off_b  = self.params_unormed[4]
        off_f  = self.params_unormed[5]
        o_b  = self.params_unormed[6]
        o_f  = self.params_unormed[7]
        omega  = self.params_unormed[8]
        coupling = "[5,5,5,0]"
        params = "["

        for front in range(2):
            params += "{'mu': " + str(mu_f) + ","
            params += "'o': " + str(o_f) + ","
            params += "'duty_factor': " + str(duty_f) + ","
            params += "'phase_offset': " + str(off_f) + ","
            params += "'omega': " + str(omega) + ","
            params += "'coupling': " + str(coupling) + "}, "
        for back in range(2):
            params += "{'mu': " + str(mu_b) + ","
            params += "'o': " + str(o_b) + ","
            params += "'duty_factor': " + str(duty_b) + ","
            params += "'phase_offset': " + str(off_b) + ","
            params += "'omega': " + str(omega) + ","
            params += "'coupling': " + str(coupling) + "}, "

        params += "]"
        self.cpg_conf["Controller"]["params"] = params
        self.cpg = control.CPG(self.cpg_conf)

    def act(self, t):

        return self.cpg.step(t)

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
        prev_t = -1
        while t < self.stop_time:
            # Get sim time
            t = p.get_gazebo_time()

            # Check that the gazebo timestep is larger than the CPG integration timestep
            if t - prev_t > self.cpg.dt:

                # Actuate
                command = self.act(t)
                p.actuate(command)
                
                # Record simulation sensor signal
                self.sens_sig.append(p.get_sensors().copy())
                self.imu_sig.append(p.get_imu().copy())
                self.pose.append(p.get_pose().copy())

            # Wait here not to overload the sensor vector
            time.sleep(0.001)
            prev_t = t

        p.stop()
        return 0

    def cleanup(self):

        self.sens_sig = []
        self.imu_sig = []
        self.pose = []
        self.model_conf = []
        self.norm_params = []
        self.cpg = None
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
            sys.stdout.write("[Sim Crash]\t")
            score = 1
        else:
            score = self.get_score()
        sys.stdout.write("Score = {0:.4f}".format(score))
        sys.stdout.write("\t\t(st: " + str(self.stop_time) + \
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

        to_save = {"iter": self.it, "score": self.score, "params": self.params_normed, "cpg_conf": self.cpg_conf, \
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