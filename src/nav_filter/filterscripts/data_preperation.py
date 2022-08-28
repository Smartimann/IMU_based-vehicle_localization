import pandas as pd
import numpy as np 
import config

def get_rotation_matrix(theta): 
    return np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])

def linear_coord_transformation(local_acc: np.ndarray, rot_matrix: np.ndarray):
    global_acc = np.dot(local_acc, rot_matrix)
    return global_acc

'''
'''
def prepare_data(filename): 
    '''Prepares the data to be used by the particle filter'''
    # read data
    simulation_data = pd.read_csv("../../data/" + filename+ ".csv")

    # this might be unnecessary because we dont use the direction of acceleration only the amount
    '''
    # transfer acceleration from inertial coordinate frame to global cf
    local_acc = np.stack((simulation_data['accelerometer_x'], simulation_data['accelerometer_y']), axis = 1)
    matrices = []
    # create rotation matrices
    for o in simulation_data['orientations'].values: 
        matrices.append(get_rotation_matrix(o))
    matrices = np.array(matrices)

    # apply rot mats to create global acceleration
    global_acc = []
    for i in range(len(matrices)): 
        global_acc.append(linear_coord_transformation(local_acc[i], matrices[i]))
    
    global_acc = np.array(global_acc)
    simulation_data['accelerometer_x'] = global_acc[:, 0]
    simulation_data['accelerometer_y'] = global_acc[:, 1]

    '''
    # add noise to acceleration to create measurement
    simulation_data['acc_x_noise'] = simulation_data['accelerometer_x'] + (config.sensor_std[0] * np.random.randn())  
    simulation_data['acc_y_noise'] = simulation_data['accelerometer_y'] + (config.sensor_std[1] * np.random.randn())  



    # invert the y axis to match simulations coodrinaty system
    simulation_data['positions_y'] = -simulation_data['positions_y']
    simulation_data['accelerometer_y'] = -simulation_data['accelerometer_y']
    simulation_data['acc_y_noise'] = -simulation_data['acc_y_noise']
    
    # calculate the acceleration amount for the control input
    acc_norms = []
    for i in range(len(simulation_data['accelerometer_x'])): 
        acc_norms.append(np.linalg.norm(np.array([simulation_data['accelerometer_x'][i], simulation_data['accelerometer_y'][i]])))

    simulation_data['acceleration_control_input'] = acc_norms

    return simulation_data