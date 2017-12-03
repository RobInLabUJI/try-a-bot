import pioneer3dx as p3dx
import matplotlib.pyplot as plt
plt.rcParams['figure.figsize'] = (10.0, 10.0)
import math
import numpy

def TR(x,y):
    xr = p3dx._ground_truth[0]
    yr = p3dx._ground_truth[1]
    c = math.cos(p3dx._ground_truth[2])
    s = math.sin(p3dx._ground_truth[2])
    T = numpy.array([[c, -s, xr],[s, c, yr],[0.,0.,1.]])
    P = numpy.array([x,y,[1]*len(x)])
    NP = numpy.dot(T,P)
    return NP[0,:], NP[1,:]

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
    d = p3dx.distance
    x = [-0.140,-0.122,-0.080,-0.029, 0.026, 0.078, 0.120, 0.138,\
          0.137, 0.119, 0.077, 0.026,-0.029,-0.080,-0.122,-0.140]
    z = [-0.061,-0.110,-0.145,-0.164,-0.164,-0.145,-0.110,-0.061,\
          0.155, 0.204, 0.239, 0.258, 0.258, 0.239, 0.204, 0.155]
    a = [3.1416,2.443, 2.094, 1.745, 1.396, 1.047, 0.698, 0.0,\
         0.0,-0.698,-1.047,-1.396,-1.745,-2.094,-2.443, 3.1416]
    #for i in range(16):
    for i in range(8):
        c = math.cos(a[i]-math.pi/2)
        s = math.sin(a[i]-math.pi/2)
        cp = math.cos(a[i]-math.pi/2 + math.pi/72)
        sp = math.sin(a[i]-math.pi/2 + math.pi/72)
        cm = math.cos(a[i]-math.pi/2 - math.pi/72)
        sm = math.sin(a[i]-math.pi/2 - math.pi/72)
        nx, ny = TR([-z[i]*1.2,-z[i]+d[i]*cp,-z[i]+d[i]*c,-z[i]+d[i]*cm], \
                    [-x[i]*1.2,-x[i]+d[i]*sp,-x[i]+d[i]*s,-x[i]+d[i]*sm])
        plt.fill(nx,ny,'cyan')
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
