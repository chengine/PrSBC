import numpy as np
import math
from cvxopt import matrix, solvers

### Utiliy Functions

def quadprog(P, q, G=None, h=None, options=None):
    """
    Quadratic programming problem with both linear equalities and inequalities
        Minimize      0.5 * x @ P @ x + q @ x
        Subject to    G @ x <= h
        and           A @ x = b
    """
    P, q = matrix(P), matrix(q)

    if G is not None:
        G, h = matrix(G), matrix(h)

    if options is None:
        sol = solvers.qp(P, q, G, h)
    else:
        sol = solvers.qp(P, q, G, h, options=options)

    return np.array(sol['x']).ravel()

# this function constructs the trap distribution resultant from convolution
# of two different central uniform distribution (i.e. from measurements of two robots positions)
# bot 1 : uniformly distributed between [-a,a]
# bot 2 : uniformly distributed between [-c,c]
# delta: x_bot1 - x_bot2 error between the two noisy measurements
# sigma: requred confidence level (>50%)
# Output: when sigma >.5
#         b2: <b1 whose CDF corresponds to 1-sigma
#         b1: >b2 whose CDF corresponds to sigma
#         when sigma < .5
#         b2: >b1 whose CDF corresponds to 1-sigma
#         b1: <b2 whose CDF corresponds to sigma

def trap_cdf_inv(a, c, delta, sigma):
#   a and c should be positive
    if a>c:  # [-A A] is the large one, and [-C C] is the smaller one
        A = a
        C = c
    else:
        A = c
        C = a   
    
    if (A==0) and (C==0):
        b2=delta
        b1=delta
        return b2, b1, sigma
    
    O_vec = [-(A+C) -(A-C) (A-C) (A+C)] # vector of vertices on the trap distribution pdf
    h = 1/(2*A) # height of the trap distribution
    area_seq = np.array([1/2*2*C*h, 2*(A-C)*h, 1/2*2*C*h])
    
    area_vec = np.array([area_seq[0], np.sum(area_seq[:2])])
    
    if abs(A-C) < 1e-5: # then is triangle
        # assuming sigma > 50%
            b1 = (A+C) - 2*C*np.sqrt((1-sigma)/(1-area_vec[1]))  # 1-area_vec(2) should be very close to 0.5
            b2 = -b1
            
            b1 = b1 + delta
            b2 = b2 + delta # apply shift here due to xi-xj
        
    else: # than is trap
        if sigma>area_vec[1]: # right triangle area
            b1 = (A+C)-2*C*np.sqrt((1-sigma)/(1-area_vec[1]))
            b2 = -(A+C) + 2*C*np.sqrt((1-sigma)/(1-area_vec[1]))
            
            b1 = b1 + delta
            b2 = b2 + delta # apply shift here due to xi-xj
  
        elif (sigma>area_vec[0]) and (sigma<=area_vec[1]): # in between the triangle part
            b1 = -(A-C) + (sigma - area_vec[0])/h # assuming >50%, then b1 should >0
            b2 = -b1
            
            b1 = b1 + delta
            b2 = b2 + delta      # apply shift here due to xi-xj  
            
            # note that b1 could be > or < b2, depending on whether sigma >
            # or < .5
            
        elif sigma<=area_vec[0]:
            b1 = -(A+C) + 2*C*np.sqrt(sigma/area_vec[0]) # assuming >50%, then b1 should >0
            b2 = -b1
            
            b1 = b1 + delta
            b2 = b2 + delta      # apply shift here due to xi-xj 
       
        else:  # first triangle, which is not allowed as long as we assume sigma > 50%
            print('bug! what is wrong?')
    return b2, b1, sigma

def find_inv_cdf(b2, b1, dir):
    if ((b2<0) and (b1>0)) or ((b2>0)and(b1<0)):
        print(f'distance between robots on {dir} smaller than error bound!')
        b = 0
    elif ((b1<0)and(b2<b1))or((b2<0)and(b2>b1)):
        b = b1
    elif ((b2>0)and(b2<b1))or((b1>0)and(b2>b1)):
        b = b2
    else:
        b = b1
        print(f'no uncertainty or sigma = 0.5 on {dir}')

    return b

# create_si_barrier_certificate
# Returns a single-integrator barrier certificate function ($f :
# \mathbf{R}**{2 \times N} \times \mathbf{R}**{2 \times N} \to \mathbf{R}**{2
# \times N}$).  This function takes a 2 x N, 2 x N single-integrator
# velocity and state vector, respectively, and returns a single-integrator
# velocity vector that does not induce collisions in the agents.
#% Detailed Description
#
# * BarrierGain - affects how quickly the agents can approach each other
# * SafetyRadius - affects the distance the agents maintain
#
# A good rule of thumb is to make SafetyRadius a bit larger than the agent
# itself (0.08 m for the GRITSbot).

# introduce decentralized probablistic Control Barrier Function to deal with uncertainty
# and obstacles
#Wenhao Luo (whluo12@gmail.com)
#Last modified: 5/25/2020
# Implementation
class PrCBF_dec():
    def __init__(self, opts, URandSpan=0, XRandSpan=0, obs_robot_idx_set=[], safety_radius=0.1, Confidence=1., gamma=1e4) -> None:
        pass
        self.URandSpan = URandSpan
        self.XRandSpan = XRandSpan
        self.obs_robot_idx_set = obs_robot_idx_set
        self.safety_radius = safety_radius
        self.Confidence = Confidence
        self.gamma = gamma
        self.opts = opts

    def barrier_certificate(self, dxi, x):
    #BARRIERCERTIFICATE Wraps single-integrator dynamics in safety barrier
    #certificates
    #   This function accepts single-integrator dynamics and wraps them in
    #   barrier certificates to ensure that collisions do not occur.  Note that
    #   this algorithm bounds the magnitude of the generated output to 0.1.
    #
    #   dx = BARRIERCERTIFICATE(dxi, x, safetyRadius)
    #   dx: generated safe, single-integrator inputs
    #   dxi: single-integrator synamics
    #   x: States of the agents (Noisy)
    #   safetyRadius:  Size of the agents (or desired separation distance)
        
        #parser.addParameter('XRandSpan',0); % XRandSpan should be 1 x N vectors containing vector of upper bound of the
        #parser.addParameter('XRandSpan',0);
        
        #parse(parser, args{:})
        #XRandSpan = parser.Results.XRandSpan;
        #Self.URandSpan = parser.Results.Self.URandSpan;
        
        N = dxi.shape[-1]
        
        if(N < 2):
            dx = dxi
            return dx, None
        
        if (len(self.XRandSpan)==1) and (self.XRandSpan==0):
            self.XRandSpan = np.zeros(2,N)
        
        if (len(self.URandSpan)==1) and (self.URandSpan==0):
            self.URandSpan = np.zeros(2,N)
        
        x = x[:3, :]
        #robot_idx_set = setdiff(1:N, obs_robot_idx_set)
        # N = N_bot + N_bot_obs and assume bot: 1:N_bot and bot_obs: (N_bot+1):N
        
        #Generate constraints for barrier certificates based on the size of
        #the safety radius
        num_constraints = 2*math.comb(N, 2)
        
        #introduce recursive policy with "stop action"
        
        start_i = 0
        loop_flag = True # initialize
        while loop_flag:
            loop_flag = False
            A = np.zeros(num_constraints, 3*N)
            b = np.zeros(num_constraints, 1)
            count = 1
        
            for i in range((start_i+1),(N-len(self.obs_robot_idx_set))):
                for j in range((i+1), N):
                    
                    max_dvij_x = np.linalg.norm(self.URandSpan(1,i)+self.URandSpan(1,j))
                    max_dvij_y = np.linalg.norm(self.URandSpan(2,i)+self.URandSpan(2,j))
                    max_dvij_z = np.linalg.norm(self.URandSpan(3,i)+self.URandSpan(3,j))

                    max_dxij_x = np.linalg.norm(x(1,i)-x(1,j)) + np.linalg.norm(self.XRandSpan(1,i)+self.XRandSpan(1,j))
                    max_dxij_y = np.linalg.norm(x(2,i)-x(2,j)) + np.linalg.norm(self.XRandSpan(2,i)+self.XRandSpan(2,j))
                    max_dxij_z = np.linalg.norm(x(3,i)-x(3,j)) + np.linalg.norm(self.XRandSpan(3,i)+self.XRandSpan(3,j))
                    
                    BB_x = -self.safety_radius**2-2/self.gamma*max_dvij_x*max_dxij_x
                    BB_y = -self.safety_radius**2-2/self.gamma*max_dvij_y*max_dxij_y
                    BB_z = -self.safety_radius**2-2/self.gamma*max_dvij_z*max_dxij_z
                    
                    [b2_x, b1_x, sigma] = trap_cdf_inv(self.XRandSpan(1,i), self.XRandSpan(1,j), x(1,i)-x(1,j), self.Confidence)
                    [b2_y, b1_y, sigma] = trap_cdf_inv(self.XRandSpan(2,i), self.XRandSpan(2,j), x(2,i)-x(2,j), self.Confidence)
                    [b2_z, b1_z, sigma] = trap_cdf_inv(self.XRandSpan(3,i), self.XRandSpan(3,j), x(3,i)-x(3,j), self.Confidence)
                    
                    # for x
                    #ratio_x = np.sqrt((self.XRandSpan(1,i)+self.XRandSpan(1,j))**2+(self.XRandSpan(2,i)+self.XRandSpan(2,j))**2)/(self.XRandSpan(1,i)+self.XRandSpan(1,j))
                    #ratio_y = np.sqrt((self.XRandSpan(1,i)+self.XRandSpan(1,j))**2+(self.XRandSpan(2,i)+self.XRandSpan(2,j))**2)/(self.XRandSpan(2,i)+self.XRandSpan(2,j))

                    #!ratio_z = np.sqrt((self.XRandSpan(1,i)+self.XRandSpan(1,j))**2+(self.XRandSpan(2,i)+self.XRandSpan(2,j))**2)/(self.XRandSpan(2,i)+self.XRandSpan(2,j))
                    ratio = 1
                    
                    b_x = find_inv_cdf(b2_x, b1_x, 'x')

                    b_y = find_inv_cdf(b2_y, b1_y, 'y')

                    b_z = find_inv_cdf(b2_z, b1_z, 'z')
                                
                    e_vec = np.array([b_x, b_y, b_z])

                    velx_vec = np.array([max_dvij_x, 0, 0])
                    vely_vec = np.array([0, max_dvij_y, 0])
                    velz_vec = np.array([0, 0, max_dvij_z])

                    x_vec = np.array([max_dxij_x, 0, 0])
                    y_vec = np.array([0, max_dxij_y, 0])
                    z_vec = np.array([0, 0, max_dxij_z])

                    base_h = np.linalg.norm(e_vec)**2-3*self.safety_radius**2-2*np.linalg.norm(velx_vec)* \
                                np.linalg.norm(x_vec)/self.gamma-2*np.linalg.norm(vely_vec)* \
                                np.linalg.norm(y_vec)/self.gamma - 2*np.linalg.norm(velz_vec)* \
                                np.linalg.norm(z_vec)/self.gamma

                    # Constraints between robots and obstacles
                    if j<(N-len(self.obs_robot_idx_set))+1:
                        A[2*count-1, (3*i-1):(3*i)] = -2*e_vec
                        A[2*count, (3*j-1):(3*j)] =  2*e_vec
                        h = base_h

                        b[(2*count-1):(2*count)] = 1/2*self.gamma*h #**3

                    # Inter-robot constraints
                    else:
                        A[2*count-1, (2*i-1):(2*i)] = -2*e_vec
                        #     A(2*count, (2*j-1):(2*j)) =  2*([b_x;b_y])'
                        h = -2*e_vec.T @ dxi[:,j]/self.gamma + base_h

                        b[(2*count-1)] = self.gamma*h #**3                    
                    
                    count += 1
            
            #Solve QP program generated earlier
            vhat = np.reshape(dxi,(2*N,1))
            H = 2*np.eye(2*N)
            f = -2*vhat
            
            vnew = quadprog(H, f, A, b, opts=self.opts)
            
            #Set robot velocities to new velocities
            if len(vnew) == 0: # if no solution exists, then iteratively set u_i=0 from i = 1,..N, until solution found
                loop_flag = True
                start_i += 1
            else:
                dx = np.reshape(vnew, (2, N))
                loop_flag = False
                dx[:,:start_i] = np.zeros(2,start_i)

        return dx