import ann
import FORCE
import numpy as np
from tigrillo_ctrl import utils

class Experiment():
	def __init__(self, ann_file, force_file, experiment_info):

		
		self.ANN = ann.ReservoirNet(config=ann_file)
		self.Force = FORCE.FORCE_Gradual(config=force_file)
		self.timestep, self.recorded_target, self.recorded_sensors = self.load(experiment_info)

		return

	def load(self,filename):
		exp_info = np.load(utils.CONFIG_FOLDER + "/" +filename).item()
	    	return exp_info['timestep'], exp_info['recorded_target'], exp_info['recorded_sensors']
