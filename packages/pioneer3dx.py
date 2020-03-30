from controller import *

import math, time

_isInitialized = False

def init():
    """Initialization.

    Args:
    """  
    global _robot
    global _leftMotor
    global _rightMotor
    global _timestep
    global _isInitialized
    global _leftEncoder
    global _rightEncoder
    global _compass
    global _gps
    
    if not _isInitialized:
        _isInitialized = True
        _robot = Robot()
        _timestep = int(_robot.getBasicTimeStep())
        _leftMotor = _robot.getMotor('left wheel')
        _rightMotor = _robot.getMotor('right wheel')
        _leftMotor.setPosition(float('+inf'))
        _rightMotor.setPosition(float('+inf'))
        _leftEncoder = _robot.getPositionSensor('left wheel sensor')
        _leftEncoder.enable(_timestep)
        _rightEncoder = _robot.getPositionSensor('right wheel sensor')
        _rightEncoder.enable(_timestep)
        _compass = _robot.getCompass('compass')
        _compass.enable(_timestep)
        _gps = _robot.getGPS('gps')
        _gps.enable(_timestep)
        move(0,0)
    
def move(ls,rs):
    """ Make the robot move.

    Args:
        ls (float): left wheel speed in rad/s
        rs (float): right wheel speed in rad/s
    """
    global _isInitialized

    if not _isInitialized:
        print('Not initialized, please call the function "p3dx.init()".')
    else:
        _leftMotor.setVelocity(ls)
        _rightMotor.setVelocity(rs)
        _robot.step(_timestep)
        
def sleep(t):
    """ Program pause (the robot keeps moving if the speeds are not zero).

    Args:
        t (float): seconds
    """
    time.sleep(t)
    
def stop():
    """ Stop the robot.

    Args:
    """
    move(0,0)
    
def encoders():
    if not _isInitialized:
        print('Not initialized, please call the function "p3dx.init()".')
        return (float('NaN'), float('NaN'))
    else:
        _robot.step(_timestep)
        return (_leftEncoder.getValue(), _rightEncoder.getValue())

def pose():
    if not _isInitialized:
        print('Not initialized, please call the function "p3dx.init()".')
        return (float('NaN'), float('NaN'), float('NaN'))
    else:
        _robot.step(_timestep)
        position = _gps.getValues()
        north = _compass.getValues()
        bearing = math.atan2(north[0], north[2]);
        return (position[0], position[2], bearing)
