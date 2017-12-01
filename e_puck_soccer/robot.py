import rospy
import datetime
import cv2
import threading
import math

from std_msgs.msg import String
from sensor_msgs.msg import Range, Image, MagneticField, NavSatFix
from geometry_msgs.msg import Twist, Vector3
from webots_ros.srv import robot_set_time_step, sensor_enable, differential_wheels_set_speed
from cv_bridge import CvBridge, CvBridgeError

class ePuck():
    
    def _psCallback(self,data):
        n = int(data.header.frame_id[-1])
        self.psValues[n] = int(data.range)
    
    def _gsCallback(self,data):
        n = int(data.header.frame_id[-1])
        self.gsValues[n] = int(data.range)
    
    def _camCallback(self,data):
		try:
			self._cv_image = self._bridge.imgmsg_to_cv2(data,"bgra8")
			self.image = cv2.cvtColor(self._cv_image, cv2.COLOR_BGR2RGB)
			self._hsv_image = cv2.cvtColor(self.image,cv2.COLOR_RGB2HSV)
			self._cvBridgeError = False
		except CvBridgeError as e:
			self._cvBridgeError = True
    
    def _gpsCallback(self,data):
        self.position = (data.latitude,-data.longitude)
        x = self._xgoal
        y = 0
        self.goal = self._cartesianToPolar(x,y)
    
    def _opponentCallback(self,data):
    	x = data.latitude
    	y = -data.longitude
    	self.opponent = self._cartesianToPolar(x,y)
    	
    def _compassCallback(self,data):
        self.orientation = math.atan2(data.magnetic_field.z,data.magnetic_field.x) + math.pi/2
        if self.orientation > math.pi:
            self.orientation -= 2*math.pi
    
    def _cartesianToPolar(self,x,y):
		try:
			dx = x - self.position[0]
			dy = y - self.position[1]
			r = math.sqrt(dx*dx+dy*dy)
			a = math.atan2(dy,dx) - self.orientation
			if a < math.pi:
				a += 2*math.pi
			if a > math.pi:
				a -= 2*math.pi
		except AttributeError:
			r = None
			a = None
		return (r,a)
    	
    def _ballCallback(self,data):
    	x = data.x
    	y = -data.z
    	#dx = x - self.position[0]
    	#dy = y - self.position[1]
    	#r = math.sqrt(dx*dx+dy*dy)
    	#a = math.atan2(dy,dx) - self.orientation
    	#if a < math.pi:
    	#	a += 2*math.pi
    	#if a > math.pi:
    	#	a -= 2*math.pi
    	#self.ball = (r,a)
    	self.ball = self._cartesianToPolar(x,y)
    	
    def _update(self):
    	r = rospy.Rate(self._rate)
    	while self._running:
    		self._timeStepClient(self._TIME_STEP)
    		r.sleep()
    
    def terminate(self):
    	self._running = False
    	while self._thread.is_alive():
    		pass
    	
    def __init__(self,name='epuck_1'):
    	try:
        	rospy.init_node('listener', anonymous=True)
        except rospy.ROSException:
        	pass
        self.controllerName = name
        #controllerName = None
        if name=='epuck_1':
        	opponent = 'epuck_2'
        	self._xgoal = -1.5
        else:
        	opponent = 'epuck_1'
        	self._xgoal = 1.5
    
        NB_DIST_SENS = 8
        NB_GROUND_SENS = 3
        SubDistIr = [None] * NB_DIST_SENS
        self.psValues = [0] * NB_DIST_SENS
        SubGndIr = [None] * NB_GROUND_SENS
        self.gsValues = [0] * NB_GROUND_SENS
    
        self._TIME_STEP = 32 # milliseconds
        self._rate = 1000 / float(self._TIME_STEP)
        self._timeStepClient = rospy.ServiceProxy(self.controllerName+"/robot/time_step",robot_set_time_step)
        self._thread = threading.Thread(None,self._update)
        
        for i in range(NB_DIST_SENS):
            deviceName = "ps"+str(i)
            enableSensorClient = rospy.ServiceProxy(self.controllerName+'/'+deviceName+'/enable',sensor_enable)
            enableSensorClient(self._TIME_STEP)
            SubDistIr[i] = rospy.Subscriber(self.controllerName+'/'+deviceName+'/value',Range,self._psCallback)
        for i in range(NB_GROUND_SENS):
            deviceName = "gs"+str(i)
            enableSensorClient = rospy.ServiceProxy(self.controllerName+'/'+deviceName+'/enable',sensor_enable)
            enableSensorClient(self._TIME_STEP)
            SubGndIr[i] = rospy.Subscriber(self.controllerName+'/'+deviceName+'/value',Range,self._gsCallback)
       
        #subscribeTwistClient = rospy.ServiceProxy(self.controllerName+"/differential_wheels/subscribe_twist_commands",differential_wheels_subscribe_twist_commands)
        #subscribeTwistClient(1)
        #self._publisher = rospy.Publisher(self.controllerName+"/differential_wheels/twist_commands",Twist,queue_size=100)
        self._diffWheelsClient = rospy.ServiceProxy(self.controllerName+"/differential_wheels/set_speed",differential_wheels_set_speed)
        
        enableCameraClient = rospy.ServiceProxy(self.controllerName+'/camera/enable',sensor_enable)
        enableCameraClient(self._TIME_STEP)
        self._bridge = CvBridge()
        SubCam = rospy.Subscriber(self.controllerName+'/camera/image',Image,self._camCallback)
        
        enableGPSClient = rospy.ServiceProxy(self.controllerName+'/gps/enable',sensor_enable)
        enableGPSClient(self._TIME_STEP)
        subGPS = rospy.Subscriber(self.controllerName+'/gps/values',NavSatFix,self._gpsCallback)
        
        rospy.Subscriber(opponent+'/gps/values',NavSatFix,self._opponentCallback)

        enableCompassClient = rospy.ServiceProxy(self.controllerName+'/compass/enable',sensor_enable)
        enableCompassClient(self._TIME_STEP)
        subCompass = rospy.Subscriber(self.controllerName+'/compass/values',MagneticField,self._compassCallback)
        
        subBall = rospy.Subscriber('/ball',Vector3,self._ballCallback)

    #def _sleep(self,t):
    #	now = rospy.get_time()
    #    endTime = now + t
    #    r = rospy.Rate(self._rate)
    #    while rospy.get_time() < endTime:
    #        #self.timeStepClient(self.TIME_STEP)
    #        r.sleep()
            
    def move(self,leftSpeed,rightSpeed,t=None):
        #command = Twist()
        #command.linear.x = (rightSpeed + leftSpeed) / 2
        #command.angular.z = (rightSpeed - leftSpeed) / 2
        #self._publisher.publish(command)
        #self._sleep(t)
        if t is None:
        	t = float(self._TIME_STEP) / 1000
        self._diffWheelsClient(leftSpeed,rightSpeed)
        rospy.sleep(t)
        
    