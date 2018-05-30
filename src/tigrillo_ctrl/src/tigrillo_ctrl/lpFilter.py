import numpy as np
from scipy.signal import butter, lfilter, freqz
import matplotlib.pyplot as plt


def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y



class LPF():
    def __init__(self, order=3, fs=50.0, cutoff=3.0, N_sensors=4):

        # Filter requirements.
        self.order = order
        self.fs = fs       # sample rate, Hz
        self.cutoff = cutoff  # desired cutoff frequency of the filter, Hz

        # Get the filter coefficients so we can check its frequency response.
        self.b, self.a = butter_lowpass(self.cutoff, self.fs, self.order)
        self.history = np.zeros((N_sensors,15))

    def plot_frequency_response(self):
        # Plot the frequency response.
        w, h = freqz(self.b, self.a, worN=8000)
        plt.subplot(2, 1, 1)
        plt.plot(0.5*self.fs*w/np.pi, np.abs(h), 'b')
        plt.plot(self.cutoff, 0.5*np.sqrt(2), 'ko')
        plt.axvline(self.cutoff, color='k')
        plt.xlim(0, 0.5*self.fs)
        plt.title("Lowpass Filter Frequency Response")
        plt.xlabel('Frequency [Hz]')
        plt.grid()

    def filterit(self, latest):
        self.history[:,0:-1] = self.history[:,1:]
        self.history[:,-1] = np.array(latest)
        filtered = []
        for x in self.history:
            filtered.append(butter_lowpass_filter(x, self.cutoff, self.fs, self.order)[-1])
        return filtered
