from controller import Supervisor

import rospy
from std_srvs.srv import Empty

s = Supervisor()

def handle_restart_sim(req):
	s.simulationRevert()

rospy.init_node('sim_supervisor')
rospy.Service('restart_sim', Empty, handle_restart_sim)

while s.step(32)!=-1:
	pass

