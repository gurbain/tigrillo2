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


amp_front    = 1955.0266213482
amp_back     = 1917.0506133149
d0           = 0.8352375546
d1           = 0.5740384878
offset_front = 19.6774000857
offset_back  = -3.4598875792
omega        = 6.28

o_fl         = 0
o_fr         = 4.3080793864
o_bl         = 1.745401654
o_br         = 2.8427800803


params = "[{'mu': " + str(amp_front) + ","
params += "'o': " + str(offset_front) + ","
params += "'duty_factor': " + str(d0) + ","
params += "'phase_offset': " + str(o_fl) + ","
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

with open("bounding.pkl", "wb") as f:
	pickle.dump(cpg_conf, f)