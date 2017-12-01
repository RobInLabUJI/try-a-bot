import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from webots_ros.srv import robot_set_time_step, sensor_enable, differential_wheels_subscribe_twist_commands

class ePuck():
	""" Mindstorms rover robot.
	
	Attributes:
		gsValues (List[int]): 3 infrared sensors pointing to the ground
		psValues (List[int]): 8 infrared sensors measuring proximity of obstacles in a range of 4 cm
	"""

	def _callback(self,data):
		self._controllerName = data.data
	
	def _psCallback(self,data):
		n = int(data.header.frame_id[-1])
		self.psValues[n] = int(data.range)
    
	def _gsCallback(self,data):
		n = int(data.header.frame_id[-1])
		self.gsValues[n] = int(data.range)
    
	def __init__(self):
		rospy.init_node('listener', anonymous=True)
		self._controllerName = None
		sb = rospy.Subscriber("/model_name", String, self._callback)
		while self._controllerName is None:
			rospy.sleep(1.0)
		sb.unregister()

		#self._timeStepClient = rospy.ServiceProxy(self._controllerName+"/robot/time_step",robot_set_time_step)
		self._TIME_STEP = 32 # milliseconds
		self._rate = 1000 / float(self._TIME_STEP)
		
		NB_DIST_SENS = 8
		self._SubDistIr = [None] * NB_DIST_SENS
		self.psValues = [0] * NB_DIST_SENS
		for i in range(NB_DIST_SENS):
			deviceName = "ps"+str(i)
			enableSensorClient = rospy.ServiceProxy(self._controllerName+'/'+deviceName+'/enable',sensor_enable)
			enableSensorClient(self._TIME_STEP)
			self._SubDistIr[i] = rospy.Subscriber(self._controllerName+'/'+deviceName+'/value',Range,self._psCallback)
        
		NB_GROUND_SENS = 3
		self._SubGndIr = [None] * NB_GROUND_SENS
		self.gsValues = [0] * NB_GROUND_SENS
		for i in range(NB_GROUND_SENS):
			deviceName = "gs"+str(i)
			enableSensorClient = rospy.ServiceProxy(self._controllerName+'/'+deviceName+'/enable',sensor_enable)
			enableSensorClient(self._TIME_STEP)
			self._SubGndIr[i] = rospy.Subscriber(self._controllerName+'/'+deviceName+'/value',Range,self._gsCallback)

		subscribeTwistClient = rospy.ServiceProxy(self._controllerName+"/differential_wheels/subscribe_twist_commands",differential_wheels_subscribe_twist_commands)
		subscribeTwistClient(1)
		self._publisher = rospy.Publisher(self._controllerName+"/differential_wheels/twist_commands",Twist,queue_size=100)

	def move(self,leftSpeed,rightSpeed,t=0.032):
		""" Make the robot move for an amount of time.
		
		Args:
			leftSpeed (int): left motor velocity (-1000, +1000)
			rightSpeed (int): right motor velocity (-1000, +1000)
			t (Optional[float]): time in seconds (default 32 ms)
		"""
		if leftSpeed > 1000:
			leftSpeed = 1000
		if leftSpeed < -1000:
			leftSpeed = -1000
		if rightSpeed > 1000:
			rightSpeed = 1000
		if rightSpeed < -1000:
			rightSpeed = -1000
		
		command = Twist()
		command.linear.x = (rightSpeed + leftSpeed) / 2
		command.angular.z = (rightSpeed - leftSpeed) / 2
		self._publisher.publish(command)
		try:
			self._sleep(t)
		except KeyboardInterrupt as e:
			m = 'the user stopped the execution'
			
			
			raise KeyboardInterrupt(m)
		
		
		except rospy.ServiceException:
			m = 'after restarting the simulation, the robot object must be redefined'
		
		
			raise rospy.ServiceException(m)



        
	def _sleep(self,t):
		rospy.sleep(t)
		#now = rospy.get_time()
		#endTime = now + t
		#r = rospy.Rate(self._rate)
		#while rospy.get_time() < endTime:
		#	self._timeStepClient(self._TIME_STEP)
		#	r.sleep()
