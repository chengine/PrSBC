import numpy as np
from scipy.integrate import odeint
from scipy.spatial import distance

class Dynamics():
    def __init__(self, x0, dt) -> None:
        # Implemented Single Integrator Dynamics in 3D
        self.x0 = x0
        self.dt = dt
        self.time = 0

    def step(self, effort):
        '''
        tspan = np.linspace(self.time, self.time+self.dt, num=10)
        sol = odeint(self.dynamics, self.x0, tspan, args=(effort,))
        self.x0 = sol[-1, :]
        self.time += self.dt
        '''
        self.x0 += effort*self.dt
        
        return self.x0

    def get_state(self):
        return self.x0

    def dynamics(self, y, t, effort):
        dxdt = effort
        return dxdt

    def return_model(self, pos):
        f = np.zeros((3,))
        g = np.eye(3)
        return f, g

    def get_inter_robot_dist(self, find_min=False):
        inter_dist = distance.cdist(self.x0.T, self.x0.T, 'euclidean')
        if find_min is True:
            m = inter_dist.shape[0]
            strided = np.lib.stride_tricks.as_strided
            s0,s1 = inter_dist.strides
            inter_dist = strided(inter_dist.ravel()[1:], shape=(m-1,m), strides=(s0+s1,s1)).reshape(m,-1)
            inter_dist = np.amin(inter_dist, axis=1)
        return inter_dist
    '''
    def dynamics(x, t, effort):
        delx = x[3:]
        delv = effort
        dxdt = np.concatenate((delx, delv), axis=0)
        return dxdt
    def return_model(self, pos):
        f = np.concatenate((pos[3:], np.zeros((3))), axis=0)
        g = np.concatenate((np.zeros((3, 3)), np.eye(3)), axis=0)
        return f, g
    '''