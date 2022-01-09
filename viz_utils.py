import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from itertools import product, combinations

def cuboid_data(o, size=np.array([1,1,1])):
    # code taken from
    # https://stackoverflow.com/a/35978146/4124317
    # suppose axis direction: x: to left; y: to inside; z: to upper
    # get the length, width, and height
    l, w, h = size
    x = [[o[0], o[0] + l, o[0] + l, o[0], o[0]],  
         [o[0], o[0] + l, o[0] + l, o[0], o[0]],  
         [o[0], o[0] + l, o[0] + l, o[0], o[0]],  
         [o[0], o[0] + l, o[0] + l, o[0], o[0]]]  
    y = [[o[1], o[1], o[1] + w, o[1] + w, o[1]],  
         [o[1], o[1], o[1] + w, o[1] + w, o[1]],  
         [o[1], o[1], o[1], o[1], o[1]],          
         [o[1] + w, o[1] + w, o[1] + w, o[1] + w, o[1] + w]]   
    z = [[o[2], o[2], o[2], o[2], o[2]],                       
         [o[2] + h, o[2] + h, o[2] + h, o[2] + h, o[2] + h],   
         [o[2], o[2], o[2] + h, o[2] + h, o[2]],               
         [o[2], o[2], o[2] + h, o[2] + h, o[2]]]               
    return np.array(x), np.array(y), np.array(z)

def plotCubeAt(pos=np.array([0,0,0]), size=np.array([1,1,1]), ax=None,**kwargs):
    # Plotting a cube element at position pos
    if ax !=None:
        X, Y, Z = cuboid_data( pos-size/2, size )
        ax.plot_surface(X, Y, Z, rstride=1, cstride=1, **kwargs)

def draw_cubes(ax, positions, sizes, colors):
    for p,s,c in zip(positions,sizes,colors):
        plotCubeAt(pos=p, size=s, ax=ax, color=c, alpha=0.2, antialiased=True, linewidth=3, edgecolor='black')

# draw sphere
def draw_sphere(ax, origin, size):
    u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
    x = size[0]*np.cos(u)*np.sin(v) + origin[0]
    y = size[1]*np.sin(u)*np.sin(v) + origin[1]
    z = size[2]*np.cos(v) + origin[2]
    ax.plot_wireframe(x, y, z, color="r")

# draw a point
def draw_point(ax, point, color="b"):
    ax.scatter(point[0], point[1], point[2], color=color, s=100)

def draw_agents(ax, positions, sizes, colors, r=None):
    for p,s,c in zip(positions,sizes,colors):
        #plotCubeAt(pos=p, size=s, ax=ax, color=c, alpha=0.2, antialiased=True, linewidth=3, edgecolor='black')
        if r is None:
            draw_sphere(ax, p, s/2)
        else:
            draw_sphere(ax, p, [r, r, r])
        #draw_point(ax, p, color=c)

class Viz():
    def __init__(self, sizes, colors, radius, dir, plt_ranges=None) -> None:
        self.sizes = sizes
        self.colors = colors
        self.radius = radius
        self.dir = dir
        self.ranges = plt_ranges

        #plt.ion()
        self.fig = plt.figure()
        self.ax = plt.axes(projection='3d')

        self.iter = 0

    def reset(self):
        self.ax.clear()
        if self.ranges is not None:
            self.ax.set_xlim(self.ranges[0])
            self.ax.set_ylim(self.ranges[1])
            self.ax.set_zlim(self.ranges[2])
    
    def plot_agents(self, positions, save=False):
        self.reset()
        draw_agents(self.ax, positions, self.sizes, self.colors, r=self.radius)
        #plt.pause(0.05)
        if save is True:
            self.save_fig()

    def save_fig(self):
        plt.savefig(self.dir + f'/{self.iter}.png')
        self.iter += 1

if __name__ == "__main__":
    positions = np.array([[-3,5,-2],[1,7,1]])
    sizes = np.array([[4,5,3], [3,3,7]])
    colors = ["crimson","limegreen"]

    fig = plt.figure()
    ax = plt.axes(projection='3d')

    '''
    draw_cubes(ax, positions, sizes, colors)
    draw_sphere(ax, positions[0], sizes[0]/2)
    draw_point(ax, positions[0])
    '''

    draw_agents(ax, positions, sizes, colors)
    plt.show()