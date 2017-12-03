import pioneer3dx as p3dx
import ipywidgets as widgets
from IPython.display import display

import rospy
from std_msgs.msg import Empty

def encCallback(data):
    global wl
    global wr
    wl.value = " Left Encoder: " + "{:7.3f}".format(p3dx.leftEncoder) + " radians"
    wr.value = "Right Encoder: " + "{:7.3f}".format(p3dx.rightEncoder) + " radians"

rs = rospy.Subscriber(p3dx._controllerName+'/encoder_updated',Empty, encCallback)
wl = widgets.Text(value='',disabled=True)
wr = widgets.Text(value='',disabled=True)

def unregister():
    global rs
    rs.unregister()
    
display(widgets.Text(value='Wheel Speeds',disabled=True))

@widgets.interact(left=(-5,5),right=(-5,5))
def motion(left,right):
    p3dx.move(left,right)

display(wl)
display(wr)