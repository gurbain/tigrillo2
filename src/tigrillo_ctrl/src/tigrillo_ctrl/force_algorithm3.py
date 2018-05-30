
import numpy as np
import pickle

class FORCE(object):
    def __init__(self, initial_weights, N_readouts, alpha):#, target
        """

        :param target: target readout value every 20ms
        :param N_readouts: number of readouts, used to check matrix dimensions
        :param initial_weights: initial w_out of the network, shape (n_res, n_out)
        """

        self.alpha = alpha
        self.N_readouts = N_readouts

        if initial_weights is not None:
            self.w = initial_weights
            self.res_size = initial_weights.shape[0]
            self.P = np.identity(self.res_size) / self.alpha
            if self.w.shape != (self.res_size, self.N_readouts):
                raise Exception("unexpected self.w shape [expected ({},{})]".format(self.res_size, self.N_readouts))
        return

    def set_initial_weights(self, initial_weights):
        """
        Allows the initial weights to be set after object initialization
        :param initial_weights:
        :return:
        """
        if hasattr(self, 'w'):
            raise Exception('tried to initialize twice!')
        else:
            self.w = initial_weights
            self.res_size = initial_weights.shape[0]
            self.P = np.identity(self.res_size) / self.alpha

        return

    def step(self, X, readout, target):
        """
        update weights and return new readout

        :param X: reservoir state (res_size, 1)
        :param readout: (1, N_readouts)
        :param target: (1, N_readouts)
        :return: output (1, N_readouts)
        """
        error = self.update_w(X, readout, target)
        output = np.dot(np.transpose(X), self.w)  # readout using new weights

        if output.shape != readout.shape:
            raise Exception("unexpected output shape")

        return output, self.w, error


    def update_w(self, X, readout, target):
        """
        update weights using RLS algorithm
        w(t) = w(t-dt) - e(t)*P(t)*X(t)
            with    e = error, distance to target
                    P = RLS matrix
                    X = reservoir states (Population Activities)
        :return:
        """

        #check dimensions
        if X.shape != (self.res_size,1):
            raise Exception("unexpected X shape [expected ({},{})]".format(self.res_size, 1))
        if readout.shape != (1,self.N_readouts):
            raise Exception("unexpected readout shape [expected ({},{})]".format(1, self.N_readouts))
        if target.shape != (1,self.N_readouts):
            raise Exception("unexpected target shape [expected ({},{})]".format(1, self.N_readouts))
        if self.w.shape != (self.res_size,self.N_readouts):
            raise Exception("unexpected self.w shape [expected ({},{})]".format(self.res_size, self.N_readouts))


        X = np.mat(X)

        e = self.get_error(readout, target)

        # print "P before"
        # print self.P
        self.update_P(X)
        if self.P.shape != (self.res_size,self.res_size):
            raise Exception("self.P shape")


        # print "P after"
        # print self.P
        self.w = self.w-e*np.array(self.P*X)

        # print "e*P*X = "
        # print e*np.array(self.P*X)
        #Tracer()()

        if self.w.shape != (self.res_size, self.N_readouts):
            raise Exception("self.w shape")

        return e


    def get_error(self, readout, target):
        """
        calculate distance to target

        e = w*X - target
          = readout - target

        :param X:
        :return:
        """

        e = readout - target

        return e

    def update_P(self, X):
        #Tracer()()
        P_prev = np.mat(self.P)
        den = 1 + X.T *P_prev * X
        num = P_prev * X * np.transpose(X) * P_prev
        """
        time0 = time.time()

        print 'num took ' + str(time.time() - time0) + ' s'

        time0 = time.time()
        one = P_prev * X
        print 'one took ' + str(time.time() - time0) + ' s'

        time0 = time.time()
        two = one * np.transpose(X)
        print 'two took ' + str(time.time() - time0) + ' s'

        Tracer()()
        time0 = time.time()
        three = two * P_prev
        print 'three took ' + str(time.time() - time0) + ' s'

        time0 = time.time()
        four = np.dot(two,P_prev)
        print 'four took ' + str(time.time() - time0) + ' s'
        """

        self.P = P_prev - num / den

        return

    def get_target(self):
        return self.target


    def load(self,filename):
        f = open(filename, 'rb')
        tmp_dict = pickle.load(f)
        f.close()

        self.__dict__.update(tmp_dict)

    def save(self, filename):
        f = open(filename, 'wb')
        pickle.dump(self.__dict__, f, 2)
        f.close()


class FORCE_Gradual(FORCE):
    def __init__(self, initial_weights=None, N_readouts=None, alpha=None, delay=None, FORCE_dur=None, post_learn=None,config=None):
        if config != None:
            self.load(config)

        else:
            FORCE.__init__(self,initial_weights, N_readouts, alpha)

            self.delay = delay
            self.FORCE_dur = FORCE_dur
            self.post_learn = post_learn

            stop = 0.95  # after 'stop' % gradual force will be ended
            self.C = stop / self.FORCE_dur  # conversion Constant, X% readout after S s of simulation

        return

    def step(self, X, readout, target, i):

        if i < self.delay + self.FORCE_dur + self.post_learn: # update weights and readout
            readout, new_w, error = super(FORCE_Gradual, self).step(X, readout, target)
        else:
            new_w = None
            error = None

        ### Calculate fraction of readout to be forwarded
        if i < self.delay:
            F = 0.0
        elif i < self.FORCE_dur + self.delay:
            F = self.C * (i - self.delay)  # Fraction of readout (0 at t = delay, allow starting up period)
        else:
            F = 1.0
        output = (F * np.array(readout)) + ((1 - F) * target)

        return output, new_w, error
