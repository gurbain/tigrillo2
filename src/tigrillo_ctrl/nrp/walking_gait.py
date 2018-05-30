""" This script use values optimized in the NRP to produce a pkl configuration 
file to use in a open-loop CPG"""

import pickle
from tigrillo_ctrl import control

__author__ = "Gabriel Urbain" 
__copyright__ = "Copyright 2017, Human Brain Projet, SP10"

__license__ = "MIT" 
__version__ = "2.0" 
__maintainer__ = "Gabriel Urbain"
__email__ = "gabriel.urbain@ugent.be" 
__status__ = "Research" 
__date__ = "April 10th, 2018"


cpg_conf = control.cpg_config

runtime = 20
cpg_conf["Controller"]["runtime"] = 20


amp_front    = 1999.3792022624
amp_back     = 1837.9431695635
o_fr         = 4.0352272465 # p0
o_bl         = 0.3475388624
o_br         = 3.2733492247
d0           = 0.8494214357
d1           = 0.5890023762
offset_front = 30.4883062863
offset_back  = -0.3721909567
omega        = 6.28

o_f          = 0

params = "[{'mu': " + str(amp_front) + ","
params += "'o': " + str(offset_front) + ","
params += "'duty_factor': " + str(d0) + ","
params += "'phase_offset': " + str(0) + ","
params += "'omega': " + str(omega) + ","
params += "'coupling': [0, 5, 5, 5]}, "

params += "{'mu': " + str(amp_front) + ","
params += "'o': " + str(offset_front) + ","
params += "'duty_factor': " + str(d0) + ","
params += "'phase_offset': " + str(o_fr) + ","
params += "'omega': " + str(omega) + ","
params += "'coupling': [5, 0, 5, 5]}, "

params += "{'mu': " + str(amp_back) + ","
params += "'o': " + str(offset_back) + ","
params += "'duty_factor': " + str(d1) + ","
params += "'phase_offset': " + str(o_bl) + ","
params += "'omega': " + str(omega) + ","
params += "'coupling': [5, 5, 0, 5]}, "

params += "{'mu': " + str(amp_back) + ","
params += "'o': " + str(offset_back) + ","
params += "'duty_factor': " + str(d1) + ","
params += "'phase_offset': " + str(o_br) + ","
params += "'omega': " + str(omega) + ","
params += "'coupling': [5, 5, 5, 0]}]"

cpg_conf["Controller"]["params"] = params

with open("walking.pkl", "wb") as f:
	pickle.dump(cpg_conf, f)