import pioneer3dx as p3dx
import matplotlib.pyplot as plt
plt.rcParams['figure.figsize'] = (10.0, 10.0)
import math
import numpy

def TR(x,y,i=-1):
    xr = p3dx._x[i]
    yr = p3dx._y[i]
    c = math.cos(p3dx._th[i])
    s = math.sin(p3dx._th[i])
    T = numpy.array([[c, -s, xr],[s, c, yr],[0.,0.,1.]])
    P = numpy.array([x,y,[1]*len(x)])
    NP = numpy.dot(T,P)
    return NP[0,:], NP[1,:]

def plot_trajectory():
    x = [-0.140,-0.122,-0.080,-0.029, 0.026, 0.078, 0.120, 0.138,\
          0.137, 0.119, 0.077, 0.026,-0.029,-0.080,-0.122,-0.140]
    z = [-0.061,-0.110,-0.145,-0.164,-0.164,-0.145,-0.110,-0.061,\
          0.155, 0.204, 0.239, 0.258, 0.258, 0.239, 0.204, 0.155]
    xp = [-xi*0.8 for xi in z]
    xp.append(xp[0])
    zp = [-zi*0.8 for zi in x]
    zp.append(zp[0])
    for i in range(0,len(p3dx._x),15):
        nx, ny = TR(xp,zp,i)
        plt.plot(nx,ny,'gray')
    
def plot_box(x,y,c):
    plt.fill([x-0.3,x+0.3,x+0.3,x-0.3],\
             [y+0.3,y+0.3,y-0.3,y-0.3],c)

def plot():
    plt.clf()
    plt.plot(-1.18,0.35,'ko')
    #plt.hold('on')
    plt.grid(True)
    ticks = numpy.arange(-4, 4, 2)
    plt.gca().set_xticks(ticks)                                                       
    plt.gca().set_yticks(ticks) 
    for xb,yb,c in [(3.7,0.0,'orange'), (3.1,0.0,'brown'), (2.0,3.7,'brown'), (1.4,3.7,'brown'), \
                  (0.8,3.7,'brown'), (0.2,3.7,'brown'), (0.2,3.1,'brown'), (0.8,3.1,'brown'), \
                  (0.7,3.3,'brown'), (1.2,-3.7,'brown'), (1.2,-3.1,'brown'), (1.2,-2.5,'brown'), \
                  (1.8,-2.5,'brown'), (-3.7,-1.3,'brown'), (-3.1,-1.3,'brown'), \
                  (-2.5,-1.3,'brown'), (-1.9,-1.3,'orange'), (-1.3,-1.3,'brown')]:
        plot_box(xb,yb,c)
    plot_trajectory()
    d = p3dx.distance
    x = [-0.140,-0.122,-0.080,-0.029, 0.026, 0.078, 0.120, 0.138,\
          0.137, 0.119, 0.077, 0.026,-0.029,-0.080,-0.122,-0.140]
    z = [-0.061,-0.110,-0.145,-0.164,-0.164,-0.145,-0.110,-0.061,\
          0.155, 0.204, 0.239, 0.258, 0.258, 0.239, 0.204, 0.155]

    xp = [-xi*0.8 for xi in z]
    xp.append(xp[0])
    zp = [-zi*0.8 for zi in x]
    zp.append(zp[0])
    nx, ny = TR(xp,zp)
    plt.plot(nx,ny,'black')
    plt.fill(nx[7:],ny[7:],'gray')
    plt.axis([-4, 4, -4, 4])
    plt.gca().set_aspect('equal', adjustable='box')
    #plt.hold('off')
