from controller import Supervisor
import sys
import math
import rospy
from geometry_msgs.msg import Vector3

class RobotSupervisor (Supervisor) :

  def run(self):
    robot_node = self.getFromDef("BALL")
    trans_field = robot_node.getField("translation")
    #count = 0
    rospy.init_node('supervisor', anonymous=True)
    pub = rospy.Publisher('ball', Vector3, queue_size=10)
    pos = Vector3()
    timeStep = int(self.getBasicTimeStep())
    winner = None
    while True:
      trans = trans_field.getSFVec3f()
      pos.x = trans[0]
      pos.y = trans[1]
      pos.z = trans[2]
      pub.publish(pos)
      if winner is None and math.fabs(pos.x) > 1.5 and math.fabs(pos.z) < 0.27:
      	print ("GOAL!!!!")
      	if pos.x < 0:
      		winner = 1
      		print "Player 1 wins!!!"
      	else:
      		winner = 2
      		print "Player 2 wins!!!"
      #if count < 10: 
      #	count +=1
      #else:
      #	print(trans)
      #	count = 0
      if self.step(timeStep) == -1:
        break

controller = RobotSupervisor()
controller.run()
