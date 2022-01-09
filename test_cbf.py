import numpy as np
from prcbf_utils import PrCBF_dec

np.random.seed(0)

#TODO: Make Configurations as a separate file
#Number of Agents
N = 7
Confidence = 0.8
safety_radius = 0.2

#Maximum noise (absolute) added to position for each agent
#x_rand_span_x = 0.02*np.random.randint(3, 4, size=(1, N))
x_rand_span_x = np.array([0.0800000000000000,
	0.0800000000000000,
    	0.0600000000000000,
        	0.0800000000000000,	0.0800000000000000,	0.0600000000000000,	0.0600000000000000])
x_rand_span_y = np.array([0.0600000000000000,	0.0800000000000000,	0.0800000000000000,
	0.0200000000000000,	0.0800000000000000,	0.0800000000000000,	0.0400000000000000])

#x_rand_span_y = 0.02*np.random.randint(1, 4, size=(1, N))
#x_rand_span_z = 0.01*np.random.randint(1, N)

v_rand_span = 0.005*np.ones((2, N))

#obs_robot_idx = np.array([6, 7])
obs_robot_idx = [6, 7]

#opts = {'show_progress': False, 'abstol': 1e-1, 'reltol': 1e-1, 'feastol': 1e-1}
XRandSpan = np.vstack((x_rand_span_x, x_rand_span_y))
URandSpan = v_rand_span
gamma = 1e4

#unsafe_cntrl = np.random.rand(2, 6)
#pos = np.random.rand(2, 6)

unsafe_cntrl = np.array([[0.499271136638754,	-0.499271136638754,	0.317853626430550,	-0.317853626430550,	-0.347511048423738,	0.347511048423738,	0],
[-0.0269876290075002,	0.0269876290075002,	-0.385965117808525,	0.385965117808525,	-0.359494188024556,	0.359494188024556,	0.500000000000000]])

pos = np.array([[-1.3, 2.4, -1.1, 1.7, 1.6, -1.3, 0.5],
        [0, -0.2, 1.7, -1.7, 1.2, -1.8, -2],
        [0, 0, 0, 0, 0, 0, 0]])

#Initialize Barrier Certificate
prcbf = PrCBF_dec(URandSpan=URandSpan, XRandSpan=XRandSpan, obs_robot_idx_set=obs_robot_idx, \
    safety_radius=2*safety_radius, Confidence=Confidence, gamma=gamma)

effort = prcbf.barrier_certificate(unsafe_cntrl, pos)

print(effort)