import rospy
import datetime
import cv2
import threading
import math

from std_msgs.msg import String
from sensor_msgs.msg import Range, Image, MagneticField, NavSatFix
from geometry_msgs.msg import Twist, Vector3
from webots_ros.srv import robot_set_time_step, sensor_enable, differential_wheels_set_speed, motor_set_position
from cv_bridge import CvBridge, CvBridgeError

class Brick():

    def _update(self):
        r = rospy.Rate(self._rate)
        while self._running:
            self._timeStepClient(self._TIME_STEP)
            r.sleep()
    
    def terminate(self):
        self._running = False
        while self._thread.is_alive():
            pass

    def _callback(self,data):
        self.controllerName = data.data
        
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
            
    def __init__(self):
        try:
            rospy.init_node('listener', anonymous=True)
            self.controllerName = None
            sb = rospy.Subscriber("/model_name", String, self._callback)
            while self.controllerName is None:
                rospy.sleep(1.0)
            sb.unregister()
        except rospy.ROSException:
            pass
                    
        NB_DIST_SENS = 1
        NB_GROUND_SENS = 2
        SubDistIr = [None] * NB_DIST_SENS
        self.psValues = [0] * NB_DIST_SENS
        SubGndIr = [None] * NB_GROUND_SENS
        self.gsValues = [0] * NB_GROUND_SENS
    
        self._TIME_STEP = 32 # milliseconds
        self._rate = 1000 / float(self._TIME_STEP)
        self._timeStepClient = rospy.ServiceProxy(self.controllerName+"/robot/time_step",robot_set_time_step)
        self._running = True
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
            
        self._diffWheelsClient = rospy.ServiceProxy(self.controllerName+"/differential_wheels/set_speed",differential_wheels_set_speed)
        
        enableCameraClient = rospy.ServiceProxy(self.controllerName+'/pixy/enable',sensor_enable)
        enableCameraClient(self._TIME_STEP)
        self._bridge = CvBridge()
        SubCam = rospy.Subscriber(self.controllerName+'/pixy/image',Image,self._camCallback)
        
        self._effector = rospy.ServiceProxy(self.controllerName+'/loop_motor/set_position',motor_set_position)
        
    def effectorUp(self):
        self._effector(1.9)
        
    def effectorDown(self):
        self._effector(-0.1)
        
    def move(self,leftSpeed,rightSpeed,t=None):
        if t is None:
            t = float(self._TIME_STEP) / 1000
        self._diffWheelsClient(leftSpeed,rightSpeed)
        rospy.sleep(t)
