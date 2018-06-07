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

phase_offset = [3.7250470724, 0.6334234742,2.8493061809]#

amp_front    = 1000.6925809072
amp_back     = 1000.2075410257
o_fr         = 3.7250470724
o_bl         = 0.6334234742
o_br         = 2.8493061809
d0           = 0.8283148999
d1           = 0.4154217047
offset_front = 24.4065673858
offset_back  = -8.1826937714
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

