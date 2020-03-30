from controller import *

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

    if not _isInitialized:
        _isInitialized = True
        _robot = Robot()
        _timestep = int(_robot.getBasicTimeStep())
        _leftMotor = _robot.getMotor('left wheel')
        _rightMotor = _robot.getMotor('right wheel')
        _leftMotor.setPosition(float('+inf'))
        _rightMotor.setPosition(float('+inf'))
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