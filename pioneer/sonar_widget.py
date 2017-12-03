import pioneer3dx as p3dx
from ipywidgets import interact
import matplotlib.pyplot as plt
import math
import numpy

import rospy
from std_msgs.msg import Empty
from IPython import display

def TR(x,y):
    xr = p3dx.ground_truth[0]
    yr = p3dx.ground_truth[1]
    c = math.cos(p3dx.ground_truth[2])
    s = math.sin(p3dx.ground_truth[2])
    T = numpy.array([[c, -s, xr],[s, c, yr],[0.,0.,1.]])
    P = numpy.array([x,y,[1]*len(x)])
    NP = numpy.dot(T,P)
    return NP[0,:], NP[1,:]

def plot_box(x,y):
    plt.fill([x-0.3,x+0.3,x+0.3,x-0.3],\
             [y+0.3,y+0.3,y-0.3,y-0.3],'brown')

def plot(d):
    x = [-0.140,-0.122,-0.080,-0.029, 0.026, 0.078, 0.120, 0.138,\
          0.137, 0.119, 0.077, 0.026,-0.029,-0.080,-0.122,-0.140]
    z = [-0.061,-0.110,-0.145,-0.164,-0.164,-0.145,-0.110,-0.061,\
          0.155, 0.204, 0.239, 0.258, 0.258, 0.239, 0.204, 0.155]
    a = [3.1416,2.443, 2.094, 1.745, 1.396, 1.047, 0.698, 0.0,\
         0.0,-0.698,-1.047,-1.396,-1.745,-2.094,-2.443, 3.1416]
    plt.clf()
    for i in range(16):
        c = math.cos(a[i]-math.pi/2)
        s = math.sin(a[i]-math.pi/2)
        nx, ny = TR([-z[i],-z[i]+d[i]*c],[-x[i],-x[i]+d[i]*s])
        plt.plot(nx,ny,'magenta')
        #plt.plot([x[i],x[i]+d[i]*c],[-z[i],-z[i]+d[i]*s],'magenta')
        plt.hold('on')
    xp = [-xi*0.8 for xi in z]
    zp = [-zi*0.8 for zi in x]
    nx, ny = TR(xp,zp)
    plt.fill(nx,ny,'black')
    #plt.fill(xp,zp,'black')
    plot_box(3.08,3.28)
    plot_box(-3.11,1.87)
    plot_box(1.55,0.19)
    plot_box(-3.05,-2.78)
    plot_box(-2.01,-0.36)
    plot_box(3.17,-1.38)
    plt.plot(0.18,0.35,'ko')
    plt.axis([-4, 4, -4, 4])
    plt.gca().set_aspect('equal', adjustable='box')
    plt.hold('off')

def usCallback(data):
    global counter
    counter = counter + 1
    if counter == 15:
        display.clear_output(wait=True)
        plot(p3dx.distance)
        display.display(plt.gcf())
        counter = 0

counter = 0
#plot(p3dx.distance)
rs = rospy.Subscriber(p3dx.controllerName+'/sonar_updated',Empty, usCallback)

def unregister():
    global rs
    rs.unregister()
    
@interact(translate=(-5,5),rotate=(-5,5))
def motion(translate,rotate):
    l = translate - rotate
    r = translate + rotate
    p3dx.move(l,r)
