from controller import Supervisor

import rospy
from std_srvs.srv import Empty

def handle_restart_sim(req):
	s = Supervisor()
	s.simulationRevert()

rospy.init_node('sim_supervisor')
s = rospy.Service('restart_sim', Empty, handle_restart_sim)
rospy.spin()

