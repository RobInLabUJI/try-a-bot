import packages.pioneer3dx as p3dx
from ipywidgets import interact

@interact(left=(-5,5),right=(-5,5))
def motion(left,right):
    p3dx.move(left,right)
