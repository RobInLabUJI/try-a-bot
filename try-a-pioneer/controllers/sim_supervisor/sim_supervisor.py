from controller import Supervisor

import rospy
from std_msgs.msg import Empty, String

class SimSupervisor (Supervisor) :

  def restart_callback(self,msg):
    self.simulationRevert()

  def load_world_callback(self,msg):
    print('World file: %s' % msg.data)
    self.loadWorld(msg.data)

  def run(self):
    rospy.init_node('sim_supervisor', anonymous=True)
    rospy.sleep(3.0)
    rospy.Subscriber('restart_sim', Empty, self.restart_callback)
    rospy.Subscriber('load_world', String, self.load_world_callback)

    timeStep = int(self.getBasicTimeStep())
    rate = 1000 / float(timeStep)
    r = rospy.Rate(rate)
    while not rospy.is_shutdown():
      if self.step(timeStep) == -1:
        break
      r.sleep()

controller = SimSupervisor()
controller.run()

