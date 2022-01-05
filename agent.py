import torch
import numpy as np
from scipy.integrate import odeint

class Dynamics():
    def __init__(self, x0, dt) -> None:
        # Implemented Single Integrator Dynamics in 3D
        self.x0 = x0
        self.dt = dt
        self.time = 0

    def step(self, effort, return_as_tensor=False):
        tspan = np.linspace(self.time, self.time+self.dt, num=10)
        sol = odeint(self.dynamics, self.x0, tspan, args=(effort,))
        self.x0 = sol[-1, :]
        self.time += self.dt
        if return_as_tensor is True:
            return torch.tensor(self.x0)
        else:
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