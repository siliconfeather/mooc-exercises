#!/usr/bin/env python
# coding: utf-8

# In[1]:


import numpy as np

# Lateral control

# TODO: write the PID controller using what you've learned in the previous activities

# Note: y_hat will be calculated based on your DeltaPhi() and poseEstimate() functions written previously 


def PIDController(
    v_0, # assume given (by the scenario)
    y_ref, # assume given (by the scenario)
    y_hat, # assume given (by the odometry)
    prev_e_y, # assume given (by the previous iteration of this function)
    prev_int_y, # assume given (by the previous iteration of this function)
    delta_t): # assume given (by the simulator)
    """
    Args:
        v_0 (:double:) linear Duckiebot speed.
        y_ref (:double:) reference lateral pose
        y_hat (:double:) the current estiamted pose along y.
        prev_e_y (:double:) tracking error at previous iteration.
        prev_int_y (:double:) previous integral error term.
        delta_t (:double:) time interval since last call.
    returns:
        v_0 (:double:) linear velocity of the Duckiebot 
        omega (:double:) angular velocity of the Duckiebot
        e_y (:double:) current tracking error (automatically becomes prev_e_y at next iteration).
        e_int_y (:double:) current integral error (automatically becomes prev_int_y at next iteration).
    """
       
    # Tracking error
    e_y = y_ref - y_hat

    # integral of the error
    e_int_y = prev_int_y + e_y * delta_t

    # anti-windup - preventing the integral error from growing too much
    e_int_y = max(min(e_int_y,2),-2)
    

    # derivative of the error
    e_der_y = (e_y - prev_e_y) / delta_t

    # controller coefficients
    Kp = 8 # 7 #5
    Ki = 0.01 #0.2
    Kd = 45 #1 #0.1

    # PID controller for omega
    omega = Kp*e_y + Ki*e_int_y + Kd * e_der_y
    
    #print(f"\n\nDelta time : {delta_t} \nE : {np.rad2deg(e_y)} \nE int_y : {e_int_y} \nPrev e_y : {prev_e_y} \nU : {u} \nY hat: {np.rad2deg(y_hat)} \n")
    
    ###
    # TODO: these are random values, you have to implement your own PID controller in here
    #omega = np.random.uniform(-8.0, 8.0)
    #e_y = np.random.random()
    #e_int_y = np.random.random()
    
    return [v_0, omega], e_y, e_int_y  

