import pandas as pd
import numpy as np 

def get_rotation_matrix(theta): 
    return np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])

def linear_coord_transformation(local_acc: np.ndarray, rot_matrix: np.ndarray):
    global_acc = np.dot(local_acc, rot_matrix)
    return global_acc

def prepare_data(): 
    simulation_data = pd.read_csv("../../data/Sim_data_long.csv")

    local_acc = np.stack((simulation_data['accelerometer_x'], simulation_data['accelerometer_y']), axis = 1)
    matrices = []
    for o in simulation_data['orientations'].values: 
        matrices.append(get_rotation_matrix(o))
    matrices = np.array(matrices)

    global_acc = []
    for i in range(len(matrices)): 
        global_acc.append(linear_coord_transformation(local_acc[i], matrices[i]))
    
    global_acc = np.array(global_acc)
    simulation_data['accelerometer_x'] = global_acc[:, 0]
    simulation_data['accelerometer_y'] = global_acc[:, 1]
    simulation_data['positions_y'] = -simulation_data['positions_y']
    simulation_data['acc_y'] = -simulation_data['acc_y']
    simulation_data['accelerometer_y'] = -simulation_data['accelerometer_y']
    simulation_data['velocity_y'] = -simulation_data['velocity_y']
    return simulation_data