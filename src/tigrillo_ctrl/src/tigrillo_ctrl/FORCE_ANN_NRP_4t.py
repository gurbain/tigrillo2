"""
evaluation for CMAES_FORCE_RESERVOIR on NRP

4targets

"""

import matplotlib.pyplot as plt
from matplotlib.mlab import PCA
import numpy as np
import os
import rospy as ros
import time
from IPython.core.debugger import Tracer
import pickle

import force_algorithm3 as force

from tigrillo_ctrl.msg import ANNStats, WoutStats
from std_msgs.msg import Float32MultiArray

reload(force)
# import rateBasedReservoir_invI_distance
# reload(rateBasedReservoir_invI_distance)
import ANN_v1
reload(ANN_v1)

# import sys
# sys.path+=["/home/alexander/Dropbox/UGent/Code/Python/gaitsSiebe"]
# import generate_cpg_control as gcpg

class force_ann_experiment:

    def __init__(self, params, res_size, target='/home/alexander/Dropbox/UGent/Code/Python/downsampledCPG.npy',
                 HLI=False, N_sensors=4,):
        """
        :param params: parameter dictionory
	target : npy file containing target signals
	HLI : boolean indicating presence of HLI control signal (currently has to be a file called 'target'_FLAGSIGNAL.npy)
        """

        self.HLI = HLI

        spec_rad, scale_w_fb, offset_w_fb, scale_noise_fb, offset_w_res, N_toKill, FORCE_dur, delay, post_learn, alpha = \
        params['spec_rad'], params['scale_w_fb'], params['offset_w_fb'], params['scale_noise_fb'], params[
            'offset_w_res'], params['N_toKill'], params['FORCE_dur'], params['delay'], params['post_learn'], params[
            'alpha']

        # Params
        self.normalized = False  # target (and feedback) normalized?
        self.offset = 7.9249263559337848  # [7.9249263559337848, 8.2570975555084445, 44.251937095108815, 43.775840610133301]
        self.multiplier = 70.649853346595961  # [70.649853346595961, 71.31419513646685, 73.503871928848739, 72.551679662214042]

        # gradual force params
        self.FORCE_dur = 10 ** FORCE_dur
        self.delay = 10 ** delay
        self.post_learn = 10 ** post_learn
        self.alpha = 10 ** alpha
        stop = 0.95

        self.n_it = int(self.delay + self.FORCE_dur + self.post_learn + 1000)

        if self.HLI:
            input_size = 1
            self.ext_input = np.load(target.replace('.npy', '_FLAGSIGNAL.npy'))/2.
        else:
            input_size = 0

        # res_size = 30*9
        self.output_size = 4
        N_sensors = N_sensors
        # FR_sim_dur = 50.0     # ms, duration of 1 (artificial net) timestep
        # input_pulse_dur = 20.0  # NRP records sensor data every 20 ms

        # input_duration = 300000  # ms
        # input_freq = 1.44  # Hz
        # sample_freq = 50  # 1./(FR_sim_dur*0.001)*5  # Hz, frequency of IO pairs

        # if not self.normalized :
        #     y = np.load('/home/alexander/Dropbox/UGent/Code/Python/downsampledCPG.npy')
        # else:
        #     y = np.load('/home/alexander/Dropbox/UGent/Code/Python/downsampledCPG_normalized.npy')
        self.target = target
        y = np.load(self.target)
        self.y_signals = [list(x) for x in y]
        # N_readouts = len(self.y_signals)
        # print "created target signal"

        # seeds = [] #[36809, 96591, 24765, 83199, 81711]
        # for i in range(10):
        self.seed = np.random.randint(0, 99999)
        # seed = 44860

        #     seeds.append(76549)

        # for x in range(-2,-1):

        # instantiate self.ANN
        fraction_inverted = 0.5
        self.ANN = ANN_v1.ReservoirNet(n_in=input_size, n_fb=N_sensors, n_out=self.output_size,
                                                                 n_res=res_size, spec_rad=spec_rad, leakage=0.7,
                                                                 scale_bias=0.0, scale_fb=scale_w_fb, scale_noise=0.00,
                                                                 scale_fb_noise=scale_noise_fb,
                                                                 negative_weights=True,
                                                                 fraction_inverted=fraction_inverted,
                                                                 seed=self.seed)  # set input weights (default random)

        # decrease weight negativity
        self.ANN.w_res += offset_w_res
        self.ANN.w_fb += offset_w_fb
        # close autoconnections
        np.fill_diagonal(self.ANN.w_res, 0.0)
        # close half of feedback connections
        self.RNG = np.random.RandomState(self.seed)
        toKill = self.RNG.choice(range(res_size), N_toKill, replace=False)
        self.ANN.w_fb[toKill] = 0.0

        # instantiate FORCE class
        #self.Force = force.FORCE(initial_weights=self.ANN.w_out, N_readouts=self.output_size, alpha=self.alpha)
        self.Force = force.FORCE_Gradual(initial_weights=np.transpose(self.ANN.w_out), N_readouts=self.output_size, alpha=self.alpha,delay = self.delay, FORCE_dur = self.FORCE_dur, post_learn = self.post_learn)

        # self.C = stop / self.FORCE_dur  # conversion Constant, X% readout after S s of simulation
        #
        # output = np.dot(np.transpose(self.ANN.w_out), self.ANN.x)  # initial readout
        # X = np.array([])
        self.error_list = np.empty((self.output_size, 1))
        self.readout_list = np.empty((self.output_size, 1))
        self.recorded_sensors = np.empty((1,N_sensors))
        self.recorded_target = np.empty((1,self.output_size))

        # Init ros name to publish stats
        self.ros_ann_stats_pub = ros.Publisher('ann_stats', ANNStats, queue_size=2)
        self.ros_wout_stats_pub = ros.Publisher('wout_stats', WoutStats, queue_size=2)

    def step(self, input, i, target=None):
        """
        simulate one step using sensor input, update readout weights and returning self.ANN output
        :param input: np.array of shape (1,x) with sensor values of robot body, should be normalized for the ANN
        :param i : step of nrp simulation
        :return: self.ANN output
        """

        self.recorded_sensors=np.vstack((self.recorded_sensors,input))  # for Gabriel

        # update self.ANN
        # Tracer()()
        if self.HLI:
            self.ANN.update_state(u=np.array([[self.ext_input[i]]]), y=np.transpose(input))  # calculates new self.ANN.x
        else:
            self.ANN.update_state(u=0, y=np.transpose(input))  # calculates new self.ANN.x

        readout = np.transpose(np.dot(self.ANN.w_out, self.ANN.x))

        # propose new weights
        if target is None:
            target = np.transpose(np.array(self.y_signals)[0:self.output_size, i:i + 1]) # Not valid with real-time
            target = target+np.random.randn(1,target.shape[1])*2# add teacher noise

        self.recorded_target=np.vstack((self.recorded_target,target))  # for Gabriel
        # if i == 0 :
        # Tracer()()
        output_force, new_w, error = self.Force.step(self.ANN.x, readout, target, i)
        self.output_force = output_force
        self.error = error
        self.final_target = target

        if new_w is not None:
            # set new weights
            self.ANN.set_w_out(np.transpose(new_w))
            # readout = np.dot(np.transpose(new_w), self.ANN.x)
            # error = self.Force.get_error(readout, target)
        else:
            error = self.Force.get_error(readout, target)

        #if i > 2:   
        #    print self.ANN.w_out - self.wout_old
        self.wout_old = self.ANN.w_out
        # store data
        self.error_list = np.hstack((self.error_list, np.transpose(error)))
        self.readout_list = np.hstack((self.readout_list, np.transpose(readout)))

        # Publish statistics for analyzing the network activity
        self.__pub_ann_stats(input)

        return output_force

    def get_result(self):
        # calculate final cost
        readout, target = np.array(self.readout_list[:, -200:]), np.array(self.y_signals)[:, -240:]
        MD = []
        for i in range(40):
            MD.append(np.mean(abs(target[:, i:i + 200] - readout)))
        self.cost = min(MD)

        return self.cost, self.seed

    def make_directory(self):

        localTime = time.localtime()[0:6]  # used to make savefolder with timestamp
        experiment_name = "-".join([str(t) for t in localTime]) + '_' + 'ClAnnForce4t_NRPexperiment'
        self.exp_dir = os.path.join('/home/alexander/Documents/ExperimentData/', experiment_name)
        os.mkdir(self.exp_dir)
        return

    def save_results(self, exp_dir=None, timestep='NotSpecified'):
        if exp_dir == None:
            self.make_directory()
        else:
            self.exp_dir = exp_dir

        ## save ANN states
        # self.ANN.save_states(self.exp_dir)

        ## save ANN object
        self.ANN.save(self.exp_dir + '/ANN')
        self.ANN.save_NNfile(self.exp_dir + '/ANN')
        # with open(self.exp_dir + '/ANN', "wb") as f:
        #     pickle.dump(self.ANN, f)

        ## save force object
        self.Force.save(self.exp_dir + '/FORCE')

        ## save experiment info
        d1 = {'recorded_sensors': self.recorded_sensors, 'recorded_target': self.recorded_target, 'timestep':timestep}
        np.save(self.exp_dir+'/experiment_info',d1)

        ## save force data
        np.save(self.exp_dir + '/force_params',
                dict(alpha=self.alpha, y_signals_target=self.target, normalized=self.normalized, delay=self.delay,
                     FORCE_dur=self.FORCE_dur, post_learn=self.post_learn, n_it=self.n_it, seed=self.seed))



        ## save np arrays
        np.save(self.exp_dir + '/error_list', self.error_list)
        np.save(self.exp_dir + '/readout_list', self.readout_list)

        # matplotlib might result in threading error with VC...
        ## save plot
        fig = plt.figure()
        ax1 = fig.add_subplot(211)
        ax1.plot(np.transpose(self.readout_list), label="Readout, alpha: " + str(self.alpha))
        ax1.plot(self.y_signals[0][:self.n_it], label="Target")
        ax1.plot(self.y_signals[-1][:self.n_it], label="Target")
        if not self.normalized:
            ax1.vlines(self.delay, -50, 50, color='black')
            ax1.vlines(self.delay + self.FORCE_dur, -50, 50, color='red', linestyle='dashed')
            ax1.vlines(self.delay + self.FORCE_dur + self.post_learn, -50, 50, color='orange', linestyle='dashed')

        else:
            ax1.vlines(self.delay, -1, 1, color='black')
            ax1.vlines(self.delay + self.FORCE_dur, -1, 1, color='red', linestyle='dashed')
            ax1.vlines(self.delay + self.FORCE_dur + self.post_learn, -1, 1, color='orange', linestyle='dashed')
        ax1.set_ylabel('readout (degrees?)')
        # ax1.set_title('readout and targets')

        ax2 = fig.add_subplot(212, sharex=ax1)
        ax2.plot(np.transpose(self.error_list), label="error, alpha: " + str(self.alpha))
        if not self.normalized:
            ax2.vlines(self.delay, -50, 50, color='black')
            ax2.vlines(self.delay + self.FORCE_dur, -50, 50, color='red', linestyle='dashed')
            ax2.vlines(self.delay + self.FORCE_dur + self.post_learn, -50, 50, color='orange', linestyle='dashed')
        else:
            ax2.vlines(self.delay, -1, 1, color='black')
            ax2.vlines(self.delay + self.FORCE_dur, -1, 1, color='red', linestyle='dashed')
            ax2.vlines(self.delay + self.FORCE_dur + self.post_learn, -1, 1, color='orange', linestyle='dashed')
            # ax2.set_title('errors')
        ax2.set_ylabel('error')
        ax2.set_xlabel('steps')
        fig.savefig(self.exp_dir + '/result_force.png')
        # fig.savefig(self.exp_dir+'/result_force.pdf')
        plt.show()
        plt.close()
        # plt.show()

        return

    def __pub_ann_stats(self, rescaled_in):

        wout_hist_y, wout_hist_x = np.histogram(self.ANN.w_out, bins=20)
        wout_mean = np.mean(self.ANN.w_out)
        wout_min = np.min(self.ANN.w_out)
        wout_max = np.max(self.ANN.w_out)
        wout_sum = np.sum(np.abs(self.ANN.w_out))
        
        wout_x_ros = Float32MultiArray(data=wout_hist_x)
        wout_y_ros = Float32MultiArray(data=wout_hist_y)

        res_states = Float32MultiArray(data=[a+i for i, a in enumerate(self.ANN.x[0:20])])
        res_in = Float32MultiArray(data=rescaled_in[0])

        y_ann = Float32MultiArray(data=np.transpose(np.dot(self.ANN.w_out, self.ANN.x))[0])
        y_force = Float32MultiArray(data=self.output_force[0])
        y_target = Float32MultiArray(data=self.final_target[0])
        if self.error is not None:
            y_err = Float32MultiArray(data=self.error[0])
        else:
            y_err = Float32MultiArray(data=[0, 0, 0, 0])


        self.ros_ann_stats_pub.publish(y_target=y_target, y_ann=y_ann, y_force=y_force, y_err=y_err,
                                       res_states=res_states, res_in=res_in, wout_sum=wout_sum, )
        self.ros_wout_stats_pub.publish(wout_x=wout_x_ros, wout_y=wout_y_ros, wout_min=wout_min, \
                                        wout_mean=wout_mean, wout_max=wout_max,)
