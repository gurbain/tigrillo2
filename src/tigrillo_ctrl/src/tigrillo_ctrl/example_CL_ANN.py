
import lpFilter
from tigrillo_2_plugin.msg import Motors, Sensors
import FORCE_ANN_NRP_4t as FAN


params = ['spec_rad', 'scale_w_fb', 'offset_w_fb', 'scale_noise_fb', 'offset_w_res', 'N_toKill', 'FORCE_dur', 'delay', 'post_learn', 'alpha']
sol_denorm = [9.9952822022920138, 1.65, 0.25, 0.010313427017175601, 0.18, 64, 3.6, 3.6, 1.5, 0.12440507262210909] #from small cmaes search
FORCE_ANN_params = dict((x, y) for x, y in zip(params, sol_denorm))

targetfile = '/home/alexander/Dropbox/UGent/Code/Python/downsampled_cpg_calibratedT_9D.npy'
force_ann = FAN.force_ann_experiment(FORCE_ANN_params,res_size=200,target=targetfile, HLI=False)


while True:
	####################### fetch input ###########################
	msg = joint_states
	pos = [msg.FL, msg.FR, msg.BL, msg.BR]
	### normalize joint readouts to feed to ANN
	offset = 5
	multiplier = 1./25
	inputNorm = np.array([pos]) + offset #output.value + offset
	inputNorm = inputNorm * multiplier

	### low pass filter joint readouts
	inputNorm[0] = lpfilter.value.filterit(inputNorm[0])

	inputNorm[0][0] = (inputNorm[0][0]-0.2)*3  # = (inputNorm[0][1] - 0.4)*5
	inputNorm[0][1] = (inputNorm[0][1]-0.2)*3
	inputNorm[0][2] = inputNorm[0][2]*2
	inputNorm[0][3] = inputNorm[0][3]*2

	######################## step ANN ##########################
	
	output.value = Force_ann.value.step(inputNorm,step.value)
	######################### publish #########################
	
	pub.publish(run_time=t, FL=output.value[0][0], FR=output.value[0][1], BL=output.value[0][2], BR=output.value[0][3])
