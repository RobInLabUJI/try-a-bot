import pioneer3dx as p3dx
from ipywidgets import interact
    
@interact(lift=(-0.05,0.05,0.01),fingers=(0.0,0.1,0.01))
def gripper(lift,fingers):
    p3dx.gripper(lift,fingers)