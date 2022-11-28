#!/usr/bin/env python
# coding: utf-8

<<<<<<< HEAD
# In[1]:
=======
# In[ ]:
>>>>>>> upstream/daffy


# start by importing some things we will need
import cv2
import matplotlib
import numpy as np
from scipy.ndimage.filters import gaussian_filter
from scipy.stats import entropy, multivariate_normal
from math import floor, sqrt


<<<<<<< HEAD
# In[4]:
=======
# In[ ]:
>>>>>>> upstream/daffy


# Now let's define the prior function. In this case we choose
# to initialize the historgram based on a Gaussian distribution around [0,0]
def histogram_prior(belief, grid_spec, mean_0, cov_0):
    pos = np.empty(belief.shape + (2,))
    pos[:, :, 0] = grid_spec["d"]
    pos[:, :, 1] = grid_spec["phi"]
    RV = multivariate_normal(mean_0, cov_0)
    belief = RV.pdf(pos)
    return belief


<<<<<<< HEAD
# In[5]:
=======
# In[ ]:
>>>>>>> upstream/daffy


# Now let's define the predict function


def histogram_predict(belief, dt, left_encoder_ticks, right_encoder_ticks, grid_spec, robot_spec, cov_mask):
<<<<<<< HEAD
        
=======
>>>>>>> upstream/daffy
        belief_in = belief
        delta_t = dt
        
        # TODO calculate v and w from ticks using kinematics. You will need  some parameters in the `robot_spec` defined above
<<<<<<< HEAD
        
        #-------
        
        #encoder_est = robot_spec['encoder_resolution']/(2 * np.pi * robot_spec['wheel_radius'])
        
        #l_d = left_encoder_ticks /  encoder_est
        #r_d = right_encoder_ticks / encoder_est

        #delta_x = (r_d + l_d) / 2
        #delta_theta = (r_d - l_d) / robot_spec['wheel_baseline']

        #v = delta_x / delta_t # replace this with a function that uses the encoder 
        #w = delta_theta / delta_t # replace this with a function that uses the encoder
        
        ##------
    
        alpha = 2*np.pi/robot_spec['encoder_resolution'] # rotation per tick in radians 

        #delta_phi = alpha*delta_ticks # in radians
        
        #delta_ticks_left = ticks_left - prev_tick_left # delta ticks of left wheel 
        #delta_ticks_right = ticks_right - prev_tick_right # delta ticks of right wheel 

        rotation_wheel_left = alpha * left_encoder_ticks # total rotation of left wheel 
        rotation_wheel_right = alpha * right_encoder_ticks # total rotation of right wheel
                
        # How much would the wheels rotate with the above tick measurements?      
        d_left = robot_spec['wheel_radius'] * rotation_wheel_left 
        d_right = robot_spec['wheel_radius'] * rotation_wheel_right
        
        #d_A = (d_left + d_right)/2  # How much has the robot travelled?
        
        #delta_x = (r_d + l_d) / 2 # How much has the robot travelled?
        #delta_theta = (r_d - l_d) / robot_spec['wheel_baseline'] # How much has the robot rotated? # expressed in radians

        delta_x = (d_right + d_left) / 2 # How much has the robot travelled?
        delta_theta = (d_right - d_left) / robot_spec['wheel_baseline'] # How much has the robot rotated? # expressed in radians
  
        v = delta_x / delta_t # replace this with a function that uses the encoder 
        w = delta_theta / delta_t # replace this with a function that uses the encoder
        
        #--------
        
        #dt = grid_spec['d'] + v * delta_t * np.sin(grid_spec['phi'])  # replace this with something that adds the new odometry
        phi_t = grid_spec['phi'] + w * delta_t # replace this with something that adds the new odometry
        dt = grid_spec['d'] + v * delta_t * np.sin(phi_t)
=======
        v = 0.0 # replace this with a function that uses the encoder 
        w = 0.0 # replace this with a function that uses the encoder
        
        # TODO propagate each centroid forward using the kinematic function
        d_t = grid_spec['d'] # replace this with something that adds the new odometry
        phi_t = grid_spec['phi'] # replace this with something that adds the new odometry
>>>>>>> upstream/daffy

        p_belief = np.zeros(belief.shape)

        # Accumulate the mass for each cell as a result of the propagation step
<<<<<<< HEAD
        
        # TODO propagate each centroid forward using the kinematic function
        
=======
>>>>>>> upstream/daffy
        for i in range(belief.shape[0]):
            for j in range(belief.shape[1]):
                # If belief[i,j] there was no mass to move in the first place
                if belief[i, j] > 0:
                    # Now check that the centroid of the cell wasn't propagated out of the allowable range
                    if (
<<<<<<< HEAD
                        dt[i, j] > grid_spec['d_max']
                        or dt[i, j] < grid_spec['d_min']
=======
                        d_t[i, j] > grid_spec['d_max']
                        or d_t[i, j] < grid_spec['d_min']
>>>>>>> upstream/daffy
                        or phi_t[i, j] < grid_spec['phi_min']
                        or phi_t[i, j] > grid_spec['phi_max']
                    ):
                        continue
                    
                    # TODO Now find the cell where the new mass should be added
<<<<<<< HEAD
                    #i_new = i # replace with something that accounts for the movement of the robot
                    i_new = int(floor((dt[i, j] - grid_spec['d_min']) / grid_spec['delta_d']))
                    
                    #j_new = j # replace with something that accounts for the movement of the robot
                    j_new = int(floor((phi_t[i, j] - grid_spec['phi_min']) / grid_spec['delta_phi']))
=======
                    i_new = i # replace with something that accounts for the movement of the robot
                    j_new = j # replace with something that accounts for the movement of the robot
>>>>>>> upstream/daffy

                    p_belief[i_new, j_new] += belief[i, j]

        # Finally we are going to add some "noise" according to the process model noise
        # This is implemented as a Gaussian blur
        s_belief = np.zeros(belief.shape)
        gaussian_filter(p_belief, cov_mask, output=s_belief, mode="constant")

        if np.sum(s_belief) == 0:
            return belief_in
<<<<<<< HEAD
        
=======
>>>>>>> upstream/daffy
        belief = s_belief / np.sum(s_belief)
        return belief


<<<<<<< HEAD
# In[6]:
=======
# In[ ]:
>>>>>>> upstream/daffy


# We will start by doing a little bit of processing on the segments to remove anything that is behing the robot (why would it be behind?)
# or a color not equal to yellow or white

def prepare_segments(segments):
    filtered_segments = []
    for segment in segments:

        # we don't care about RED ones for now
        if segment.color != segment.WHITE and segment.color != segment.YELLOW:
            continue
        # filter out any segments that are behind us
        if segment.points[0].x < 0 or segment.points[1].x < 0:
            continue

        filtered_segments.append(segment)
    return filtered_segments


<<<<<<< HEAD
# In[7]:
=======
# In[ ]:
>>>>>>> upstream/daffy


def generate_vote(segment, road_spec):
    p1 = np.array([segment.points[0].x, segment.points[0].y])
    p2 = np.array([segment.points[1].x, segment.points[1].y])
    t_hat = (p2 - p1) / np.linalg.norm(p2 - p1)
    n_hat = np.array([-t_hat[1], t_hat[0]])
    
    d1 = np.inner(n_hat, p1)
    d2 = np.inner(n_hat, p2)
    l1 = np.inner(t_hat, p1)
    l2 = np.inner(t_hat, p2)
    if l1 < 0:
        l1 = -l1
    if l2 < 0:
        l2 = -l2

    l_i = (l1 + l2) / 2
    d_i = (d1 + d2) / 2
    phi_i = np.arcsin(t_hat[1])
    if segment.color == segment.WHITE:  # right lane is white
        if p1[0] > p2[0]:  # right edge of white lane
            d_i -= road_spec['linewidth_white']
        else:  # left edge of white lane
            d_i = -d_i
            phi_i = -phi_i
        d_i -= road_spec['lanewidth'] / 2

    elif segment.color == segment.YELLOW:  # left lane is yellow
        if p2[0] > p1[0]:  # left edge of yellow lane
            d_i -= road_spec['linewidth_yellow']
            phi_i = -phi_i
        else:  # right edge of white lane
            d_i = -d_i
        d_i = road_spec['lanewidth'] / 2 - d_i

    return d_i, phi_i


<<<<<<< HEAD
# In[8]:
=======
# In[ ]:
>>>>>>> upstream/daffy


def generate_measurement_likelihood(segments, road_spec, grid_spec):

    # initialize measurement likelihood to all zeros
    measurement_likelihood = np.zeros(grid_spec['d'].shape)

    for segment in segments:
        d_i, phi_i = generate_vote(segment, road_spec)

        # if the vote lands outside of the histogram discard it
        if d_i > grid_spec['d_max'] or d_i < grid_spec['d_min'] or phi_i < grid_spec['phi_min'] or phi_i > grid_spec['phi_max']:
            continue

        # TODO find the cell index that corresponds to the measurement d_i, phi_i
<<<<<<< HEAD
        #i = 1 # replace this
        i = int(floor((d_i - grid_spec['d_min']) / grid_spec['delta_d']))
        
        #j = 1 # replace this
        j = int(floor((phi_i - grid_spec['phi_min']) / grid_spec['delta_phi']))
=======
        i = 1 # replace this
        j = 1 # replace this
>>>>>>> upstream/daffy
        
        # Add one vote to that cell
        measurement_likelihood[i, j] += 1

    if np.linalg.norm(measurement_likelihood) == 0:
        return None
    measurement_likelihood /= np.sum(measurement_likelihood)
    return measurement_likelihood


<<<<<<< HEAD
# In[9]:
=======
# In[ ]:
>>>>>>> upstream/daffy


def histogram_update(belief, segments, road_spec, grid_spec):
    # prepare the segments for each belief array
    segmentsArray = prepare_segments(segments)
    # generate all belief arrays

    measurement_likelihood = generate_measurement_likelihood(segmentsArray, road_spec, grid_spec)

    if measurement_likelihood is not None:
        # TODO: combine the prior belief and the measurement likelihood to get the posterior belief
        # Don't forget that you may need to normalize to ensure that the output is valid probability distribution
<<<<<<< HEAD
        
        belief = np.multiply(belief, measurement_likelihood) # replace this with something that combines the belief and the measurement_likelihood            
        if np.sum(belief) == 0:
            belief = measurement_likelihood
        else:    
            belief = belief / np.sum(belief)
                             
=======
        belief = measurement_likelihood # replace this with something that combines the belief and the measurement_likelihood
>>>>>>> upstream/daffy
    return (measurement_likelihood, belief)

