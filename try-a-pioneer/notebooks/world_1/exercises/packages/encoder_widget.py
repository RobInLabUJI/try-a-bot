import pioneer3dx as p3dx
import ipywidgets as widgets
from IPython.display import display

import rospy
from std_msgs.msg import Empty

def encCallback(data):
    global w
    w.value = "Encoders: " + "{:.3f}".format(p3dx.leftEncoder) + " " + "{:.3f}".format(p3dx.rightEncoder)

rs = rospy.Subscriber(p3dx._controllerName+'/encoder_updated',Empty, encCallback)
w = widgets.Text(value='',disabled=True)

def unregister():
    global rs
    rs.unregister()
    
display(widgets.Text(value='Wheel Speeds',disabled=True))

@widgets.interact(left=(-5,5),right=(-5,5))
def motion(left,right):
    p3dx.move(left,right)

display(w)