import numpy as np
import os
import time
import tqdm
import json
import matplotlib.pyplot as plt
from agent import Dynamics
from planner import Planner
from PrCBF_utils import PrCBF_dec

np.random.seed(0)

def run():
    
    #TODO: Make Configurations as a separate file
    #Number of Agents
    N = 7
    Confidence = 0.8
    safety_radius = 0.2

    #Maximum noise (absolute) added to position for each agent
    x_rand_span_x = 0.02*np.random.rand(1, N)
    x_rand_span_y = 0.02*np.random.rand(1, N)
    x_rand_span_z = 0.01*np.random.rand(1, N)

    v_rand_span = 0.005*np.ones(3, N)

    x0 = np.array([[-1.3, 2.4, -1.1, 1.7, 1.6, -1.3, 0.5],\
        [0, -0.2, 1.7, -1.7, 1.2, -1.8, -2],\
        [0, 0, 0, 0, 0, 0, 0]])

    goal = np.hstack([x0[:2,[2, 1, 4, 3, 6, 5]], np.array([0.5, 2])])

    obs_robot_idx = np.array([6, 7])

    dt = 0.01
    iter_final = 3000
    path = 'cbf_logs/'

    #Initialize Planner
    scale = 0.1
    planner = Planner(x0, goal, scale=scale)

    #Initialize Dynamics
    dynamics = Dynamics(x0, dt)

    opts = {'show_progress': False}
    XRandSpan = np.vstack((x_rand_span_x, x_rand_span_y, x_rand_span_z))
    URandSpan = v_rand_span
    gamma = 1e4

    #Initialize Barrier Certificate
    prcbf = PrCBF_dec(opts, URandSpan=URandSpan, XRandSpan=XRandSpan, obs_robot_idx_set=obs_robot_idx, \
        safety_radius=2*safety_radius, Confidence=Confidence, gamma=gamma)

    for iter in tqdm.trange(iter_final):

        pos = dynamics.get_state()

        #Add noise to position
        pos += ...

        unsafe_cntrl = planner.get_action(pos)
        effort = prcbf.barrier_certificate(unsafe_cntrl, pos)

        #Add noise to velocities
        effort += ...

        current_state = dynamics.step(effort)
        traj = np.vstack((traj, current_state))

    fig = plt.figure()
    ax = plt.axes(projection='3d')
    traj = x0

    print('Final Position', dynamics.get_state())
    ax.plot3D(traj[:, 0], traj[:, 1], traj[:, 2], 'blue')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    plt.savefig('cbf_logs/trajectory.png')
    plt.clf()

    meta = traj.tolist()
    with open(path + 'traj.json', 'w') as f:
        json.dump(meta, f)

if __name__=='__main__':

    run()

