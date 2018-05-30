
import generate_cpg_control as gcpg
import numpy as np
import matplotlib.pyplot as plt

input_duration = 9000#00  # ms

sample_freq = 50 #Hz 
"""
## gait 1
##################5D cmaes####################
cmaes_params = ['po', 'd0','d1', 'offset_front', 'offset_back']
cmaes_result = [3.2413127551, 0.211397468, 0.7326430427, -21.1273863722, -44.701767989] #2017-9-18-19-42-32_smallTigrillo_V1_0
#cmaes_result = [2.2790555049, 0.1956551467, 0.7788317559, -56.4503996947, -46.4655112539] #2017-9-19-23-28-58_tigrillo_scaled_2_D5N30
#cmaes_result = [2.0340744571, 0.1632550915, 0.7898942269, -57.9799255258, -53.0439114074] #2017-9-19-23-28-58_tigrillo_scaled_2_D5N30/iteration45
cmaes_dict = dict((x,y) for x, y in zip(cmaes_params,cmaes_result))

#for small tigrillo V1
mu = [1247.8506096107708, 1271.4286080336062, 1350.704882789273, 1315.9368951279903] # amplitude
o = [cmaes_dict['offset_front'], cmaes_dict['offset_front'], cmaes_dict['offset_back'], cmaes_dict['offset_back']] # offset
omega = [8.966, 8.966, 8.966, 8.966]
d = [cmaes_dict['d0'], cmaes_dict['d0'], cmaes_dict['d1'], cmaes_dict['d1']] # duty cycle
phase_offset = [0.0, cmaes_dict['po'], cmaes_dict['po']]
params1=[mu, o, omega, d, phase_offset]
cpg = gcpg.CPGControl(mu, o, omega, d, phase_offset)
#####################################
"""
"""
##################7D cmaes####################
cmaes_params = ['po0', 'po1', 'po2', 'd0','d1', 'offset_front', 'offset_back']
#cmaes_result = [2.5207115564, 3.2030357947, 0.5635350325, 0.1215260447, 0.4374383347, -50.6735687191, -52.8367739961] #2017-9-19-12-15-43_smallTigrillo_V1_0_test_freePhases/iteration26
#cmaes_result = [4.1009682881,3.2466187818, 0.0061914678, 0.3975706851, 0.5102393513, -31.7045448732, -55.4275270132] #2017-9-19-12-15-43_smallTigrillo_V1_0_test_freePhases/iteration26
#cmaes_result = [3.117216713, 3.5805672076, 0.4983965045, 0.2018984599, 0.431241239, -50.5287942639, -58.1467273582] #2017-9-19-12-15-43_smallTigrillo_V1_0_test_freePhases/iteration26
#cmaes_result = [4.956428689, 0.8693965091, 1.0057709612, 0.3565657469, 0.4991825631, -57.9081051751, -59.009645482] #2017-9-19-12-15-43_smallTigrillo_V1_0_test_freePhases/iteration26
cmaes_result = [3.429805552, 3.0847672897, 0.0425837786, 0.1315289634, 0.7607450554, -1.2052280883, -59.6837517889]  #2017-9-20-23-36-4_tigrillo_scaled_feets_D7N30/2017-9-21-8-18-10_iteration86
cmaes_dict = dict((x,y) for x, y in zip(cmaes_params,cmaes_result))

C=0.2
#for small tigrillo V1
mu = [1247.8506096107708, 1271.4286080336062, 1350.704882789273, 1315.9368951279903] # amplitude
o = [cmaes_dict['offset_front'], cmaes_dict['offset_front'], cmaes_dict['offset_back'], cmaes_dict['offset_back']] # offset
omega = [8.966*C, 8.966*C, 8.966*C, 8.966*C]
d = [cmaes_dict['d0'], cmaes_dict['d0'], cmaes_dict['d1'], cmaes_dict['d1']] # duty cycle
phase_offset = [cmaes_dict['po0'], cmaes_dict['po1'], cmaes_dict['po2']]
#####################################
"""
"""
#real Tigrillo 13D
cmaes_params = ['amp_front', 'amp_back',  'po0','po1','po2','d0L','d0R','d1L','d1R', 'offset_front_L', 'offset_front_R', 'offset_back_L', 'offset_back_R']
cmaes_result = [1995.7099789934, 1329.1776800352, 6.1912688963, 3.4014967152,	6.1735169153, 0.1167378107, 0.7280241508, 0.3020831716, 0.31960611, 50.4405367733, 57.0642534081, -12.9733777616, -10.8132940565] #sftp://paard.elis.ugent.be/home/nrp/Documents/ExperimentData/2018-2-14-18-52-4_CMAES-CPG_CMAES_RealTigrillo_13D/2018-2-15-3-43-35_iteration177/
cmaes_dict = dict((x,y) for x, y in zip(cmaes_params,cmaes_result))

mu = [cmaes_dict['amp_front'], cmaes_dict['amp_front'] , cmaes_dict['amp_back'], cmaes_dict['amp_back']] #amplitude?
o = [cmaes_dict['offset_front_L'], cmaes_dict['offset_front_R'], cmaes_dict['offset_back_L'], cmaes_dict['offset_back_R']] #offset
omega = [8.966, 8.966, 8.966, 8.966 ]#frequencyish 8.966
d = [cmaes_dict['d0L'],cmaes_dict['d0R'],cmaes_dict['d1L'],cmaes_dict['d1R']] #duty cycle
phase_offset = [cmaes_dict['po0'], cmaes_dict['po1'], cmaes_dict['po2']]
#####################################
"""
"""
#real Tigrillo 9D
cmaes_params = ['amp_front', 'amp_back',  'po0','po1','po2','d0','d1', 'offset_front', 'offset_back']
cmaes_result = [668.25574929834215, 1822.3681372677133, 3.7880702189186857, 0.60736980037586219, 3.6471465072591993,0.59685632758599882, 0.10100339905535813, 54.730753979237697, -7.4285040557806914] #sftp://paard.elis.ugent.be/home/nrp/Documents/ExperimentData/2018-2-15-17-31-46_CMAES-CPG_CMAES_RealTigrillo_9D/2018-2-16-3-36-48_iteration199
cmaes_dict = dict((x,y) for x, y in zip(cmaes_params,cmaes_result))

mu = [cmaes_dict['amp_front'], cmaes_dict['amp_front'] , cmaes_dict['amp_back'], cmaes_dict['amp_back']] # amplitude
o = [cmaes_dict['offset_front'], cmaes_dict['offset_front'], cmaes_dict['offset_back'], cmaes_dict['offset_back']] # offset
omega = [8.966, 8.966, 8.966, 8.966]
d = [cmaes_dict['d0'], cmaes_dict['d0'], cmaes_dict['d1'], cmaes_dict['d1']] # duty cycle
phase_offset = [cmaes_dict['po0'], cmaes_dict['po1'], cmaes_dict['po2']]
cpg = gcpg.CPGControl(mu, o, omega, d, phase_offset)
#####################################

"""

"""
#####################################
#7D RealT cmaes bounding gait
cmaes_params = ['amp_front', 'amp_back', 'po', 'd0', 'd1', 'offset_front', 'offset_back']
cmaes_result = [2000.7145318283, 1940.7274052927,	3.1386621172,	0.1035151832, 0.1172187076, 59.9359462891, 15.4933086429] #/home/alexander/Documents/ExperimentData/2018-2-19-0-51-7_CMAES-CPG_realTigrillo_searchBoundingGait/2018-2-19-8-39-34_iteration149 ->bounding gait
cmaes_dict = dict((x,y) for x, y in zip(cmaes_params,cmaes_result))

mu1 = [cmaes_dict['amp_front'], cmaes_dict['amp_front'], cmaes_dict['amp_back'], cmaes_dict['amp_back']] # amplitude
o1 = [cmaes_dict['offset_front'], cmaes_dict['offset_front'], cmaes_dict['offset_back'], cmaes_dict['offset_back']] # offset
omega1 = [8.966, 8.966, 8.966, 8.966]
d1 = [cmaes_dict['d0'], cmaes_dict['d0'], cmaes_dict['d1'], cmaes_dict['d1']] # duty cycle
phase_offset1 = [0, cmaes_dict['po'], cmaes_dict['po']]
params1=[mu1, o1, omega1, d1, phase_offset1]
cpg = gcpg.CPGControl(mu1, o1, omega1, d1, phase_offset1)
#####################################
"""
"""
#####################################
#7D RealT cmaes bounding gait
cmaes_params = ['amp_front', 'amp_back', 'po', 'd0', 'd1', 'offset_front', 'offset_back']
cmaes_result = [357.7145318283, 1940.7274052927,	3.1386621172,	0.1035151832, 0.1172187076, 59.9359462891, 15.4933086429] #/home/alexander/Documents/ExperimentData/2018-2-19-0-51-7_CMAES-CPG_realTigrillo_searchBoundingGait/2018-2-19-8-39-34_iteration149 ->bounding gait
cmaes_dict = dict((x,y) for x, y in zip(cmaes_params,cmaes_result))

mu1 = [10.,200.,300.,1000.] # amplitude
o1 = [cmaes_dict['offset_front'], cmaes_dict['offset_front'], cmaes_dict['offset_back'], cmaes_dict['offset_back']] # offset
#omega1 = [5., 0., 5., 0.]
#omega1 = [0., 5., 0., 5.]
omega1 = [5., 5., 5., 5.]
d1 = [cmaes_dict['d0'], cmaes_dict['d0'], 0.18, cmaes_dict['d1']] # duty cycle
phase_offset1 = [0.0, cmaes_dict['po'], cmaes_dict['po']]
params1=[mu1, o1, omega1, d1, phase_offset1]
cpg = gcpg.CPGControl(mu1, o1, omega1, d1, phase_offset1)
#####################################
"""

#####################################
#7D calibrated RealT cmaes: 
cmaes_params = ['amp_front', 'amp_back', 'po', 'd0', 'd1', 'offset_front', 'offset_back']
cmaes_result = [1479.4733148744, 1966.2014925501, 1.7907346139, 0.1132321966, 0.1000351019, 7.1551633616, -19.8186370404] #/home/alexander/Documents/ExperimentData/2018-4-25-6-47-35_CMAES-CPG_CMAES_CalibratedTigrillo_7D_1Hz/2018-4-25-10-26-35_iteration88
cmaes_dict = dict((x,y) for x, y in zip(cmaes_params,cmaes_result))
mu = [cmaes_dict['amp_front'], cmaes_dict['amp_front'], cmaes_dict['amp_back'], cmaes_dict['amp_back']] # amplitude
o = [cmaes_dict['offset_front'], cmaes_dict['offset_front'], cmaes_dict['offset_back'], cmaes_dict['offset_back']] # offset
omega = [6.28, 6.28, 6.28, 6.28]
d = [cmaes_dict['d0'], cmaes_dict['d0'], cmaes_dict['d1'], cmaes_dict['d1']] # duty cycle
phase_offset = [0, cmaes_dict['po'], cmaes_dict['po']]
cpg = gcpg.CPGControl(mu, o, omega, d, phase_offset)
#####################################
"""

## gait0
#####################################
#calibrated Tigrillo 9D
cmaes_params = ['amp_front', 'amp_back',  'po0','po1','po2','d0','d1', 'offset_front', 'offset_back']
cmaes_result = [1995.20545060220, 1999.5335596501, 2.3767353984, 6.1711704122, 2.2575967656, 0.5797908017, 0.849271304, 50.5657854418, 21.841845545] #/home/alexander/Documents/ExperimentData/2018-4-24-23-25-52_CMAES-CPG_CMAES_CalibratedTigrillo_9D_1Hz/2018-4-25-6-40-4_iteration173
cmaes_dict = dict((x,y) for x, y in zip(cmaes_params,cmaes_result))
mu = [cmaes_dict['amp_front'], cmaes_dict['amp_front'] , cmaes_dict['amp_back'], cmaes_dict['amp_back']] # amplitude
o = [cmaes_dict['offset_front']-30, cmaes_dict['offset_front']-30, cmaes_dict['offset_back']-30, cmaes_dict['offset_back']-30] # offset
omega = [6.28, 6.28, 6.28, 6.28]
d = [cmaes_dict['d0'], cmaes_dict['d0'], cmaes_dict['d1'], cmaes_dict['d1']] # duty cycle
phase_offset = [cmaes_dict['po0'], cmaes_dict['po1'], cmaes_dict['po2']]
cpg = gcpg.CPGControl(mu, o, omega, d, phase_offset)
#####################################
"""



sig = np.transpose([ cpg.step_open_loop() for i in range(input_duration)]) #siebes cpg generator presumably with sample frequency of 1000 Hz
#downsample, we only need frequency of sample_freq
downfactor = 1000/sample_freq
shape1 = sig.shape[1]/downfactor
sig2 = np.empty((sig.shape[0],shape1))
for idx, row in enumerate(sig):
    sig2[idx] = row[::downfactor][:shape1]

print "caveat, forced bounding"
sig2[1] = sig2[0]

sig2 =np.array(sig2)
plt.figure()
plt.plot(np.transpose(sig2))
plt.xlabel('timestep')
plt.ylabel('degrees')
plt.title('Target Signals')
plt.legend(loc=0)
plt.show()

y_signals = [list(x[16:]) for x in sig2]

np.save('/home/alexander/downsampled_cpg_calibratedT_7D',y_signals)
