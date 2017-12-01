from controller import Supervisor

import rospy
from std_msgs.msg import Empty

s = Supervisor()

def callback(msg):
	#print('restart signal received')
	s.simulationRevert()

rospy.init_node('sim_supervisor')
rospy.sleep(3.0)
rospy.Subscriber('restart_sim', Empty, callback)

#print('supervisor started')

TIME_STEP = 32 # milliseconds
rate = 1000 / float(TIME_STEP)
r = rospy.Rate(rate)
#while s.step(TIME_STEP)!=-1:
while not rospy.is_shutdown():
	s.step(TIME_STEP)
	r.sleep()
	

