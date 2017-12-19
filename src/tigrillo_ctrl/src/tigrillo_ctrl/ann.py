
import numpy as np
import rospy as ros
import pickle
import sklearn.linear_model

from tigrillo_ctrl import utils, control


__author__ = "Alexander Vandesompele" 
__copyright__ = "Copyright 2017, Human Brain Project, SP10"

__license__ = "MIT" 
__version__ = "1.0" 
__maintainer__ = "Gabriel Urbain"
__email__ = "gabriel.urbain@ugent.be" 
__status__ = "Research" 
__date__ = "December 7th, 2017"


class Neuron(object):

    def __init__(self):

        return

    def tran_fct(self, xStates):

        x = xStates.copy()
        for idx,itm in enumerate(x):
            if itm <= 0 :
                x[idx] = 0
            else :
                x[idx] = 0.786*itm/(0.110+itm)-0.014 #fitted IO curve
        return x


class ReservoirNet(Neuron, control.Controller):
    """
    This class implements constant and methods of a artificial neural network
    using Reservoir Computing architecture, with the goal of comparing with a network of spiking populations.
    """

    def __init__(self, n_in=0, n_out=1, n_res=100, spec_rad=1.15, leakage=0.1, scale_bias=0.5, scale_fb=5.0, scale_noise=0.01, scale_fb_noise=0.01,
                 verbose=False, negative_weights=True, fraction_inverted=0.0, seed=None, config=None):

        Neuron.__init__(self)
        control.Controller.__init__(self)

        self.scale_bias = scale_bias
        self.scale_fb = scale_fb
        self.scale_noise = scale_noise
        self.scale_feedback_noise = scale_fb_noise

        self.leakage = leakage

        self.TRANS_PERC = 0.1

        self.n_in = n_in
        self.n_out = n_out
        self.n_res = n_res

        if seed == None:
            self.rng = np.random.RandomState(np.random.randint(0,99999))
        else:
            self.rng = np.random.RandomState(seed)

        self.w_out = self.rng.randn(n_res, n_out)
        if not negative_weights:
            self.w_out = abs(self.w_out)
        self.w_in = self.rng.randn(n_res, n_in)
        if not negative_weights:
            self.w_in = abs(self.w_in)
        self.w_bias = self.rng.randn(n_res,1) * self.scale_bias
        self.w_bias = 0.0 # mimics resting noise level of approx 50 Hz
        self.w_res = self.get_rand_mat(n_res, spec_rad,negative_weights=negative_weights)

        self.w_fb = self.rng.randn(n_res, n_out) * self.scale_fb

        if not(negative_weights):
            self.w_fb = abs(self.w_fb)

        self.p_connect_res = self.createConnectivityMatrix()
        p_connect_fb = np.ones((n_res)) * 0.1
        self.p_connect_fb = p_connect_fb.reshape(-1, 1)

        self.N_inverted = int(np.round(n_res*fraction_inverted))

        self.x = np.zeros((n_res, 1))
        self.u = np.zeros(n_in)
        self.y = np.zeros(n_out)

        self.verbose = verbose

        self.Y = np.array([])
        self.X = np.array([])

        self.t = 0
        self.prev_t = 0
        self.dt = 0.01

        if config:
            self.load(config["Controller"]["filename"])
            self.dt = float(config["Controller"]["anntimestep"])

    def getCoordinates(self, ID, xD, yD):

        if not (isinstance(ID, (int, long)) & isinstance(xD, (int, long)) & isinstance(yD, (int, long))):
            raise Exception('population ID, xDimension and yDimension must be integer types')
        zD = xD * yD

        z = ID / zD
        y = (ID - z * zD) / xD
        x = ID - z * zD - y * xD

        return x, y, z

    def getProb(self, ID0, ID1, xD, yD, C=0.3, lamb=1.0):
        """
        get distance-based connection probability
        :param ID0: id of population 0
        :param ID1: id of population 1
        :param xD: x dimensionality of grid
        :param yD: y dimensionality of grid
        :param C: parameter to weight connectivity based on connection type (not yet implemented, from maass 2002)
        :param lamb: parameter to in/decrease overall connectivity
        :return: Probability of connection between any two neurons of populations ID0 and ID1
        """
        if ID0 == ID1:
            prob = 0.
        else:
            x0, y0, z0 = self.getCoordinates(ID0, xD, yD)
            x1, y1, z1 = self.getCoordinates(ID1, xD, yD)
            d = np.sqrt((x0 - x1) ** 2 + (y0 - y1) ** 2 + (z0 - z1) ** 2)
            prob = C * np.power(np.e, -np.square(d / lamb))
        return prob

    def createConnectivityMatrix(self):

        p_connect = np.empty((self.n_res,self.n_res))

        for fr in range(self.n_res):
            for to in range(self.n_res):
                p_connect[fr,to] = self.getProb(to,fr,xD=3,yD=3)

        return p_connect

    def get_rand_mat(self, dim, spec_rad,negative_weights=True):

        mat = self.rng.randn(dim, dim)
        if not(negative_weights):
            mat = abs(mat)
        w, v = np.linalg.eig(mat)
        mat = np.divide(mat, (np.amax(np.absolute(w)) / spec_rad))

        return mat

    def update_state(self, u=0, y=0):
        """
        Update the state of the reservoir
        u = input vector
        y = output vector (can be used for output or body feedback)
        """

        fb = self.w_fb * (np.ones((self.n_res, 1)) * y + self.rng.randn(self.n_res, 1) * self.scale_feedback_noise)
        fb = fb.sum(axis=1).reshape(-1,1)

        #create noise term
        noise = self.rng.randn(self.n_res, 1) * self.scale_noise

        # Reservoir update equation if no input
        if self.n_in == 0:
            # x_new = np.dot(self.w_res, self.x) + self.w_bias + np.dot(self.w_fb, y)
            # Tracer()()
            x_new = np.dot(self.w_res*self.p_connect_res, self.x) + self.w_bias + fb*self.p_connect_fb + noise

        # Reservoir update equation if input
        else:
            x_new = np.dot(self.w_res*self.p_connect_res, self.x) + self.w_bias + fb*self.p_connect_fb + np.dot(self.w_in, u) + noise
        # leakage

        self.x = (1 - self.leakage) * self.x + self.leakage * self.tran_fct(x_new)

        return

    def step(self, t, u=0, y=0):
        """
        Return the network outputs at time t
        """
        
        self.t = t
        n_steps = (int(self.t / self.dt) - self.prev_t)

        # If the ANN timestep is higher than the realtime one
        if n_steps == 0:
            return self.y

        for _ in range(n_steps):
            self.update_state(u, y)

        self.prev_t = int(self.t/self.dt)

        return np.dot(np.transpose(self.w_out), self.x)

    def run(self, n_it, U=None, fakeFB=None, to_plot=200):
        """
        Run the network for n_it timesteps given a timeserie of input vector U . \
        U = inputs with shape (n_it,n_in)
        fakeFB = np array, will be used instead of the actual network feedback (useful to test attractor state strength)
        """

        self.scale_noise = self.scale_noise/2

        if self.n_in > 0:
            if U.shape != (n_it,self.n_in):
                raise ValueError("inputs should be of shape (n_it,n_in)")

        if fakeFB == None:
            num_fb = 0
        else:
            num_fb = fakeFB.shape[0]

        print("\n -- RUNNING --")
        print(" -- Update the state matrix for " + str(n_it) + " time steps -- ")
        for j in range(n_it):

            if j < num_fb:
                y = fakeFB[j][0]
            elif j == 0:
                y = 0.0
            else:
                y = self.Y[j - 1, :].reshape(-1, 1)

            if self.n_in == 0:
                    self.update_state(y=y) # feed back the output of previous timestep/fakeFB
            else:
                    self.update_state(u=U[j].reshape(-1, 1), y=y) # feed back the output of previous timestep/fakeFB


        if self.X.min() == 0 and self.X.max() == 0:
            print "!!! WARNING: it seems there has been no activity whatsoever in the network !!!"

    def train(self, Ytrain, U=np.zeros([]), to_plot=200):
        """
        Train the network given a timeserie of input vector @U and desired output vector @Y. \
        To perform the training, the network is first updated for each input and a matrix \
        X of the network states is sampled. After removing the transiant states, \
        w_out is computed with least square
        """

        n_it = Ytrain.shape[0] - 1
        X = np.array([])
        if self.n_in > 0:
            if U.shape != (Ytrain.shape[0],self.n_in):
                raise ValueError("inputs should be of shape (Ntrainingsamples,n_in)")

        print("\n -- RUNNING to generate training data --")
        print(" -- Update the state matrix for " + str(n_it) + " time steps -- ")
        for j in range(n_it):
            if self.n_in == 0:
                self.update_state(y=Ytrain[j,:].reshape(-1, 1))  # feed back the desired output of previous timestep

            else:
                self.update_state(u=U[j].reshape(-1, 1), y=Ytrain[j,:].reshape(-1, 1))  # feed back the desired output of previous timestep

            # y = np.dot(np.transpose(self.w_out),self.x)
            # Y = np.vstack([Y, np.transpose(y)]) if Y.size else np.transpose(y)
            X = np.vstack([X, np.transpose(self.x)]) if X.size else np.transpose(self.x)


        if X.min() == 0 and X.max() == 0:
            print "!!! WARNING: it seems there has been no activity whatsoever in the network !!!"

        # store X and Y
        self.X = X.copy()
        self.Y = Ytrain.copy()

        print(" -- Removing transiant states -- ")
        to_remove = int(self.TRANS_PERC * n_it)
        X = np.delete(X, np.s_[0:to_remove], 0)
        Ytrain = np.delete(Ytrain, np.s_[0:to_remove], 0)

        print(" -- Updating w_out using linear regression -- ")
        if self.verbose == True:
            print("( inv( " + str(np.transpose(X).shape) + " X " + str(X.shape) + " ) X " + str(np.transpose(X).shape) + \
                  " ) X " + str(Ytrain.shape))

    def set_w_out(self, w_out):

        self.w_out = w_out
        return

    def save_states(self, exp_dir):

        np.save(exp_dir+'/ANN_states.npy',self.X)

    def save(self, exp_dir):

        with open(exp_dir + '/ANN', "wb") as f:
            pickle.dump(self, f)

    def load(self, file):
        """
        Populate this class via a pickle file
        """

        with open(utils.CONFIG_FOLDER + "/" + file, 'rb') as f:
            tmp_dict = pickle.load(f)
            f.close()

            self.__dict__.update(tmp_dict.__dict__)
