import numpy as np
import pandas as pd
from scipy import stats


'''
Returns the likelihood a measurement corresponds with a particle state
'''
def get_likelihood(particle_value, R, measuerment): 
    return stats.norm(particle_value, R).pdf(measuerment)



'''
Calculates the difference between to rotations by creating to vectors from this rotation with length 1 and 
returning the norm of their difference
'''
def get_rotation_difference(angle_one, angle_two): 
    #return abs(np.exp(1j*angle_one/180*np.pi) - np.exp(1j*angle_two/180*np.pi))
    return np.linalg.norm(np.array([np.cos(angle_one), np.sin(angle_one)]) - np.array([np.cos(angle_two), np.sin(angle_two)]))

'''
Returns the norm of the difference between to acc vectors
'''
def get_acceleration_difference(acc_a, acc_b): 
    acceleration_difference = np.linalg.norm(acc_a - acc_b)
    return acceleration_difference
