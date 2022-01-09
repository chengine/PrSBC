import numpy as np
import os
import time
import tqdm
import json
import matplotlib.pyplot as plt
from agent import Dynamics
from planner import Planner
from viz_utils import Viz
from PrCBF_utils import PrCBF_dec
#from prcbf_utils import PrCBF_dec
import random

np.random.seed(0)

def run():
    
    #TODO: Make Configurations as a separate file
    #Number of Agents
    N = 7
    Confidence = 0.9
    safety_radius = 0.2

    #Maximum noise (absolute) added to position for each agent
    #x_rand_span_x = 0.02*np.random.rand(1, N)
    #x_rand_span_y = 0.02*np.random.rand(1, N)
    #x_rand_span_z = 0.01*np.random.rand(1, N)

    x_rand_span_x = 0.02*np.random.randint(3, 4, size=(1, N))
    x_rand_span_y = 0.02*np.random.randint(1, 4, size=(1, N))
    x_rand_span_z = 0.01*np.random.randint(1, 3, size=(1, N))

    v_rand_span = 0.005*np.ones((3, N))

    x0 = np.array([[-1.3, 2.4, -1.1, 1.7, 1.6, -1.3, 0.5],
        [0, -0.2, 1.7, -1.7, 1.2, -1.8, -2],
        [0, 0, 0, 0, 0, 0, 0]])

    #x0 = x0[:2, :]

    goal = np.hstack((x0[:,[1, 0, 3, 2, 5, 4]], np.array([0.5, 2, 0]).reshape(-1, 1)))

    #goal = goal[:2, :]

    #print(goal, x0)

    obs_robot_idx = np.array([6, 7])
    #obs_robot_idx = []

    dt = 0.1
    iter_final = 3000
    path = 'cbf_logs/'

    #Initialize Planner
    scale = 0.1
    planner = Planner(x0, goal, scale=scale)

    #Initialize Dynamics
    dynamics = Dynamics(x0, dt)

    opts = {'show_progress': False}
    #XRandSpan = np.vstack((x_rand_span_x, x_rand_span_y, x_rand_span_z))
    XRandSpan = np.vstack((x_rand_span_x, x_rand_span_y, x_rand_span_z))
    URandSpan = v_rand_span
    gamma = 1e2

    #Initialize Barrier Certificate
    prcbf = PrCBF_dec(opts, URandSpan=URandSpan, XRandSpan=XRandSpan, obs_robot_idx_set=obs_robot_idx, \
        safety_radius=2*safety_radius, Confidence=Confidence, gamma=gamma)

    # Initialize visualization
    dir = './logs'
    r = np.random.rand(N, 1)
    b = np.random.rand(N, 1)
    g = np.random.rand(N, 1)
    colors = np.hstack((r, g, b))
    colors = colors.tolist()
    colors = [tuple(item) for item in colors]

    #plt_ranges = [(-4, 4), (-4, 4), (-1., 1.)]

    viz = Viz(XRandSpan.T, colors, safety_radius, dir)

    traj = x0

    inter_dists = []

    for iter in tqdm.trange(iter_final):

        pos = dynamics.get_state()

        #Add noise to position
        pos_error = 2*(np.random.rand(3,N)-0.5)
        pos += XRandSpan*pos_error

        unsafe_cntrl = planner.get_action(pos)
        effort = prcbf.barrier_certificate(unsafe_cntrl, pos)

        #Add noise to velocities
        #effort = unsafe_cntrl
        vel_error = 2*(np.random.rand(3,N)-0.5)
        effort += v_rand_span*vel_error

        current_state = dynamics.step(effort)
        #traj = np.vstack((traj, current_state))

        inter_dists.append(dynamics.get_inter_robot_dist(find_min=True))

        if iter % 25 == 0:
            print(current_state.T)
            viz.plot_agents(current_state.T, save=True)
    #plt.show()
    print(goal.T)

    inter_dists = np.stack(inter_dists)
    min_inter_dists = np.amin(inter_dists, axis=0)

    print(min_inter_dists)
    '''
    fig = plt.figure()
    ax = plt.axes(projection='3d')

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
    '''

if __name__=='__main__':

    run()

