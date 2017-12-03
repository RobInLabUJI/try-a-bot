import pioneer3dx as p3dx
from ipywidgets import interact
    
@interact(angle=(-0.47,0.47))
def tilt(angle):
    p3dx.tilt(angle)