import rospy

from std_msgs.msg import String
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from webots_ros.srv import robot_set_time_step, sensor_enable, differential_wheels_subscribe_twist_commands
from webots_ros.msg import BoolStamped, Float64Stamped

class Rover():
	""" Mindstorms rover robot.
	
	Attributes:
		s1 (bool): Left bumper
		s2 (int): Color sensor
		s3 (bool): Right bumper
	"""
	
	def _callback(self,data):
		self._controllerName = data.data
    
	def _s1Callback(self,data):
		self.s1 = data.data
    
	def _s2Callback(self,data):
		self.s2 = int(data.range)
    	
	def _s3Callback(self,data):
		self.s3 = data.data
    
	#def _lwCallback(self,data):
	#	self.lw = int(data.data)
    	
	#def _rwCallback(self,data):
	#	self.rw = int(data.data)
    	
	def __init__(self):
		self.s1 = False
		self.s2 = 0
		self.s3 = False
		self._controllerName = None
		self._TIME_STEP = 32 # milliseconds
		self._rate = 1000 / float(self._TIME_STEP)
		self._publisher = None
		rospy.init_node('listener', anonymous=True)
		
		sb = rospy.Subscriber("/model_name", String, self._callback)
		while self._controllerName is None:
		    rospy.sleep(1.0)
		sb.unregister()
		self._timeStepClient = rospy.ServiceProxy(self._controllerName+"/robot/time_step",robot_set_time_step)
		
		deviceName = "S1"
		enableSensorClient = rospy.ServiceProxy(self._controllerName+'/'+deviceName+'/enable',sensor_enable)
		enableSensorClient(self._TIME_STEP)
		self._s1Sub = rospy.Subscriber(self._controllerName+'/'+deviceName+'/value',BoolStamped,self._s1Callback)
		
		deviceName = "S2"
		enableSensorClient = rospy.ServiceProxy(self._controllerName+'/'+deviceName+'/enable',sensor_enable)
		enableSensorClient(self._TIME_STEP)
		self._s2Sub = rospy.Subscriber(self._controllerName+'/'+deviceName+'/value',Range,self._s2Callback)
		
		deviceName = "S3"
		enableSensorClient = rospy.ServiceProxy(self._controllerName+'/'+deviceName+'/enable',sensor_enable)
		enableSensorClient(self._TIME_STEP)
		self._s3Sub = rospy.Subscriber(self._controllerName+'/'+deviceName+'/value',BoolStamped,self._s3Callback)
		
		subscribeTwistClient = rospy.ServiceProxy(self._controllerName+"/differential_wheels/subscribe_twist_commands",differential_wheels_subscribe_twist_commands)
		subscribeTwistClient(1)
		self._publisher = rospy.Publisher(self._controllerName+"/differential_wheels/twist_commands",Twist,queue_size=100)
		
		#deviceName = "differential_wheels_encoders"
		#enableEncodersClient = rospy.ServiceProxy(self._controllerName+'/'+deviceName+'/enable',sensor_enable)
		#enableEncodersClient(self._TIME_STEP)
		#self._lwSub = rospy.Subscriber(self._controllerName+'/differential_wheels/lwheel',Float64Stamped,self._lwCallback)
		#self._rwSub = rospy.Subscriber(self._controllerName+'/differential_wheels/rwheel',Float64Stamped,self._rwCallback)
		
	def move(self,leftSpeed,rightSpeed,t=1):
		""" Make the robot move for an amount of time.
		
		Args:
			leftSpeed (int): left motor velocity (-100, +100)
			rightSpeed (int): right motor velocity (-100, +100)
			t (Optional[float]): time in seconds (default 1)
		"""
		if leftSpeed > 100:
			leftSpeed = 100
		if leftSpeed < -100:
			leftSpeed = -100
		if rightSpeed > 100:
			rightSpeed = 100
		if rightSpeed < -100:
			rightSpeed = -100
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
		now = rospy.get_time()
		endTime = now + t
		r = rospy.Rate(self._rate)
		while rospy.get_time() < endTime:
			self._timeStepClient(self._TIME_STEP)
			r.sleep()
			