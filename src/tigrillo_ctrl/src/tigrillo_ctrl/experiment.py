import ann
import FORCE

class experiment():
	def __init__(self, ann_file, force_file):
		ann = ann.ReservoirNet(config=ann_file)
		force = FORCE.FORCE_Gradual(config=force_file)
		return
