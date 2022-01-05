import numpy as np

def rotate(theta):
    mat = np.eye(3)
    mat[0, :] = np.array([np.cos(theta), np.sin(theta), 0.])
    mat[1, :] = np.array([-np.sin(theta), np.cos(theta), 0.])

    return mat

class Planner():
    def __init__(self, start, goal, scale=1., threshold=1e-4) -> None:
        self.goal = goal
        self.scale = scale
        self.x0 = start
        self.threshold = threshold
        self.theta = 0.

    def get_action(self, pos):
        thrust = self.scale*(rotate(self.theta)@self.goal - pos)/np.linalg.norm(rotate(self.theta)@self.goal - pos)
        if np.linalg.norm(pos-self.x0) <= self.threshold:
           #thrust += np.random.normal(0., 0.025, size=(3,))
            self.theta += 0.1
        else:
            self.theta -= 0.01
            if self.theta <= 0.:
                self.theta = 0.
        self.x0 = pos
        return thrust