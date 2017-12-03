import rospy as _rospy
import math as _math
import numpy as _numpy

from std_msgs.msg import String as _String, Empty as _Empty
from sensor_msgs.msg import NavSatFix as _NavSatFix, \
                            MagneticField as _MagneticField, \
                            Range as _Range, \
                            Image as _Image

from webots_ros.msg import Float64Stamped as _Float64Stamped
from webots_ros.srv import sensor_enable as _sensor_enable, \
                           motor_set_velocity as _motor_set_velocity, \
                           motor_set_position as _motor_set_position

_controllerName = None
_TIME_STEP = 32

_ground_truth = [0, 0, 0]
_isInitialized = False
distance = [0] * 8

def init():
    """Initialization.

    Args:
    """    
    global _isInitialized
    if not _isInitialized:
        _rospy.init_node('controller', anonymous=True)
        _isInitialized = True
        
    global _controllerName
    _controllerName = None
    sb = _rospy.Subscriber("/model_name", _String, _nameCallback)
    while _controllerName is None:
        _rospy.sleep(0.1)
    sb.unregister()
    global _lwv
    global _rwv
    _lwv, lwp = _enableMotor('left_wheel',0.0,float('inf'))
    _rwv, rwp = _enableMotor('right_wheel',0.0,float('inf'))
    global _tmv
    global _tmp
    _tmv, _tmp = _enableMotor('tilt_motor',0.0,0.0)

    global _enc_pbl
    _enc_pbl = _rospy.Publisher(_controllerName+'/encoder_updated',_Empty,_Empty,queue_size=100)
    _enableEncoder('left_wheel_sensor',_leftEncoder)
    _enableEncoder('right_wheel_sensor',_rightEncoder)
    _enablePose()
    _enableSonar()
    _enableCamera()

def move(ls,rs):
    """ Make the robot move.

    Args:
        ls (float): left wheel speed in rad/s
        rs (float): right wheel speed in rad/s
    """
    global _isInitialized
    if _isInitialized:
        global _lwv
        global _rwv
        max_speed = 5.24
        if abs(ls) > max_speed:
            rs = rs * max_speed / abs(ls)
            ls = ls * max_speed / abs(ls)
        if abs(rs) > max_speed:
            ls = ls * max_speed / abs(rs)
            rs = rs * max_speed / abs(rs)
        try:
            _lwv(ls)
            _rwv(rs)
            sleep(0.032)
        except _rospy.ServiceException as e:
            
            
            raise Exception('The robot needs initialization - has the simulation been restarted?')
            
            

def sleep(t):
    """ Program pause (the robot keeps moving if the speeds are not zero).

    Args:
        t (float): seconds
    """
    _rospy.sleep(t)
    
def stop():
    """ Stop the robot.

    Args:
    """
    move(0,0)
    
def tilt(a):
    """ Tilt the Kinect.

    Args:
        a (float): radians
    """
    global _tmv
    global _tmp
    _tmv(1.0)
    if a>0.47:
        a = 0.47
    if a<-0.47:
        a = -0.47
    _tmp(a)

def pose():
    print('x:  %5.2f' % _ground_truth[0])
    print('y:  %5.2f' % _ground_truth[1])
    print('th: %5.2f' % _ground_truth[2])

def _nameCallback(data):
    global _controllerName
    _controllerName = data.data

def _enableMotor(w,v_ini,p_ini):
    vp = _rospy.ServiceProxy(_controllerName+'/'+w+'/set_velocity',_motor_set_velocity)
    pp = _rospy.ServiceProxy(_controllerName+'/'+w+'/set_position',_motor_set_position)
    vp(v_ini)
    pp(p_ini)
    return vp, pp

def _enableEncoder(deviceName,cb):
    global _TIME_STEP
    sp = _rospy.ServiceProxy(_controllerName+'/'+deviceName+'/enable',_sensor_enable)
    sp(_TIME_STEP)
    sb = _rospy.Subscriber(_controllerName+'/'+deviceName+'/value',_Float64Stamped,cb)

def _leftEncoder(data):
    global leftEncoder
    leftEncoder = data.data

def _rightEncoder(data):
    global rightEncoder
    rightEncoder = data.data
    global _enc_pbl
    _enc_pbl.publish(_Empty())

def _gpsCallback(data):
    global _ground_truth
    x = data.longitude
    y = data.latitude
    _ground_truth[0] = x
    _ground_truth[1] = y

def trajectory():
    return _x,_y

def _compassCallback(data):
    global _ground_truth
    mf = data.magnetic_field
    pi = _math.pi
    bearing = _math.atan2(mf.x, mf.z)
    bearing = -bearing - pi/2
    if bearing > pi:
        bearing = bearing - 2*pi
    elif bearing < -pi:
        bearing = bearing + 2*pi
    _ground_truth[2] = bearing
    x = _ground_truth[0]
    y = _ground_truth[1]
    if len(_x) == 0:
        _x.append(x)
        _y.append(y)
        _th.append(bearing)
    elif x != _x[-1] or y != _y[-1] or bearing != _th[-1]:
        _x.append(x)
        _y.append(y)
        _th.append(bearing)
    
def _enablePose():
    global _TIME_STEP
    global _x
    global _y
    global _th
    _x = []
    _y = []
    _th = []

    sp = _rospy.ServiceProxy(_controllerName+'/gps/enable',_sensor_enable)
    sp(_TIME_STEP)
    sb = _rospy.Subscriber(_controllerName+'/gps/values',_NavSatFix,_gpsCallback)

    sp = _rospy.ServiceProxy(_controllerName+'/compass/enable',_sensor_enable)
    sp(_TIME_STEP)
    sb = _rospy.Subscriber(_controllerName+'/compass/values',_MagneticField,_compassCallback)
    
def _enableSonar():
        global _controllerName
        global _TIME_STEP
        global _so_pbl
        _so_pbl = _rospy.Publisher(_controllerName+'/sonar_updated',_Empty,queue_size=100)
        #for s in range(16):
        for s in range(8):
            deviceName = 'so'+str(s)
            sp = _rospy.ServiceProxy(_controllerName+'/'+deviceName+'/enable',_sensor_enable)
            sp(_TIME_STEP)
            _rospy.Subscriber(_controllerName+'/'+deviceName+'/value',_Range,_usCallback)
            
def _usCallback(data):
    f = data.header.frame_id
    if f[-2] == 'o':
        so = int(f[-1])
    else:
        so = int(f[-2:])
    global distance
    distance[so] = data.range
    #if so == 15:
    if so == 7:
        global _so_pbl
        _so_pbl.publish(_Empty())
        
def _camCallback(data):
    global image
    global _img_pbl
    im = _numpy.fromstring(data.data,dtype=_numpy.uint8).reshape(15000,4)
    im.shape = (-1,4)
    image = im[:,[2,1,0,3]].reshape(100,150,4)
    _img_pbl.publish(_Empty())
    
def _enableCamera():
    global _controllerName
    global _TIME_STEP
    global _img_pbl
    deviceName = 'kinectColor'
    _img_pbl = _rospy.Publisher(_controllerName+'/'+deviceName+'/image_updated',_Empty,queue_size=100)
    sp = _rospy.ServiceProxy(_controllerName+'/'+deviceName+'/enable',_sensor_enable)
    sp(_TIME_STEP)
    _rospy.Subscriber(_controllerName+'/'+deviceName+'/image',_Image,_camCallback)
