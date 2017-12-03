import pioneer3dx as p3dx
from ipywidgets import interact
    
@interact(translate=(-5,5),rotate=(-5,5))
def motion(translate,rotate):
    l = translate - rotate
    r = translate + rotate
    p3dx.move(l,r)