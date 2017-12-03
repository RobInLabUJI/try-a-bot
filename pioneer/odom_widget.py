import pioneer3dx as p3dx
from ipywidgets import interact
import matplotlib.pyplot as plt

import rospy
from std_msgs.msg import Empty
from IPython import display

def encCallback(data):
    global counter
    counter = counter + 1
    if counter == 10:
        display.clear_output(wait=True)
        #print('    Encoders: (left_wheel: %.2f, right_wheel: %.2f)' % (p3dx.leftEncoder,p3dx.rightEncoder))
        #print('Ground truth: (x: %.2f, y: %.2f, th: %.2f)' % tuple(p3dx.ground_truth))
        #print('    Odometry: (x: %.2f, y: %.2f, th: %.2f)' % tuple(p3dx.odometry))
        global xgt
        global ygt
        global xo
        global yo
        xo.append(p3dx.odometry[0])
        yo.append(p3dx.odometry[1])
        xgt.append(p3dx.ground_truth[0])
        ygt.append(p3dx.ground_truth[1])
        plt.plot(xgt,ygt,'b')
        plt.hold('on')
        plt.plot(xo,yo,'r')
        plt.axis([-4, 4, -4, 4])
        plt.gca().set_aspect('equal', adjustable='box')
        plt.legend(['ground truth','odometry'])
        display.display(plt.gcf())
        counter = 0
        
rs = rospy.Subscriber(p3dx.controllerName+'/encoder_updated',Empty, encCallback)
counter = 0

xgt = []
ygt = []
xo = []
yo = []

def unregister():
    global rs
    rs.unregister()
    
@interact(translate=(-5,5),rotate=(-5,5))
def motion(translate,rotate):
    l = translate - rotate
    r = translate + rotate
    p3dx.move(l,r)
    #display.clear_output(wait=True)
    #print((p3dx.leftEncoder,p3dx.rightEncoder))