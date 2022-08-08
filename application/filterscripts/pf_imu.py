import numpy as np
import pandas as pd
import cv2 as cv
import matplotlib.pyplot as plt
import sys
import os
from filterpy.monte_carlo import systematic_resample
import scipy as sp
from scipy import stats
from sklearn import preprocessing
from numba import jit,njit, vectorize, cuda



PROJECT_ROOT = os.path.abspath(os.path.join(
                  os.path.dirname('utils'), 
                  os.pardir)
)
sys.path.append(PROJECT_ROOT)
from filterscripts import distance_map

def update(particles, weights,z, R, dm):
    rotation_differences = abs((abs(particles[:,6]-z[2])+180) %360 - 180)# index 6 = theta
    rot_diff_weight = 1 - preprocessing.minmax_scale(rotation_differences)

    acceleration_difference = np.array(list(map(np.linalg.norm, z[0:2] - particles[:,4:5])), dtype=object)
    acc_diff_weight = 1 - preprocessing.minmax_scale(acceleration_difference)

    particles_image_coords = dm.coord_to_image(particles[:, 0:2])
    distances = []
    
    for p in particles_image_coords: 
        if (p[0] < dm.distance_map.shape[1] and p[0] > 0 and p[1] < dm.distance_map.shape[0] and p[1] > 0): 
            distances.append(dm.distance_map[p[1], [0]])
        else:
            distances.append(np.array(1000000))
    distances = np.array(distances, dtype=object)
    average_distances = rot_diff_weight + acc_diff_weight + distances / 3
    weights = weights * average_distances

    weights += 1.e-300
    
    weights /= sum(weights) # normalize
    
    
def estimate(particles, weights): 
    pos = particles[:,0:2]
    mean = np.average(pos, weights=weights, axis=0)
    var  = np.average((pos - mean)**2, weights=weights, axis=0)
    return mean, var


def resample_from_index(particles, weights, indexes):
    particles[:] = particles[indexes]
    weights.resize(len(particles))
    weights.fill (1.0 / len(weights))
    
def neff(weights):
    return 1. / np.sum(np.square(weights))

# x = x-0,y-1,x_dot-2, y_dot-3, x_ddot-4, y_ddot-5, theta-6, delta-7
# u = acc_x, acc_y, steering 
def F(x, u, step, L, std, N): 
    # calculate nect velocity
    x_dot_next = x[2] + (x[4] * step) 
    y_dot_next = x[3] + (x[5] * step)
    # control input as next acceleration
    x_ddot_next = u[0] + (np.random.uniform(0,1,1) * std[0])
    y_ddot_next = u[1] + (np.random.uniform(0,1,1) * std[1])
    # calculate velocity scalar
    velocity = np.linalg.norm((x[2], x[3]))
    # get next position
    x_next = x[0] + (velocity*np.cos(x[6] + x[7]) * step)
    y_next = x[1] + (velocity*np.sin(x[6] + x[7]) * step)
    #calculate theta and delta
    theta_next = x[6] + (((velocity*np.sin(x[7]))/L) * step)
    delta_next = x[7] + ((u[2] + np.random.uniform(0,1,1)*std[2]) * step)
    
    
    return np.array([x_next, y_next,x_dot_next, y_dot_next, x_ddot_next, y_ddot_next, theta_next, delta_next], dtype=object) 

def predict(particles, u, std, dt, L): 
    N = len(particles) 
    # Needs noise: not in a for loop
    for i in range(len(particles)): 
        particles[i] = F(particles[i], u, dt, L, std, len(particles))

from numpy.random import uniform

def create_uniform_particles(x_range, y_range, x_dot_range, y_dot_range,x_ddot_range, y_ddot_range, theta_range, delta_range,N):
    particles = np.empty((N, 8))
    particles[:, 0] = uniform(x_range[0], x_range[1], size=N)
    particles[:, 1] = uniform(y_range[0], y_range[1], size=N)
    particles[:, 2] = uniform(x_dot_range[0], x_dot_range[1], size=N)
    particles[:, 3] = uniform(y_dot_range[0], y_dot_range[1], size=N)
    particles[:, 4] = uniform(x_ddot_range[0], x_ddot_range[1], size=N)
    particles[:, 5] = uniform(y_ddot_range[0], y_ddot_range[1], size=N)    
    particles[:, 6] = uniform(theta_range[0], theta_range[1], size=N)
    particles[:, 7] = uniform(delta_range[0], delta_range[1], size=N)
    particles[:, 6] %= 2 * np.pi
    particles[:, 7] %= 2 * np.pi
    return particles


def get_rotation_matrix(theta): 
    return np.array([[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)]])

def prepare_data(): 
    simulation_data = pd.read_csv("../../data/Sim_data_long.csv")
    local_acc = np.stack((simulation_data['accelerometer_x'], simulation_data['accelerometer_y']), axis = 1)

    matrices = []
    for o in simulation_data['orientations'].values: 
        matrices.append(get_rotation_matrix(o))
    matrices = np.array(matrices)

    global_acc = []
    for i in range(len(matrices)): 
        global_acc.append(np.dot(local_acc[i], matrices[i]))
    global_acc = np.array(global_acc)
    simulation_data['accelerometer_x'] = global_acc[:, 0]
    simulation_data['accelerometer_y'] = global_acc[:, 1]
    simulation_data['positions_y'] = -simulation_data['positions_y']
    simulation_data['acc_y'] = -simulation_data['acc_y']
    simulation_data['accelerometer_y'] = -simulation_data['accelerometer_y']
    simulation_data['velocity_y'] = -simulation_data['velocity_y']
    return simulation_data

def run_pf_imu(simulation_data, sensor_std, std):
    dm = distance_map.DistanceMap(1, 300, 'road_points_data_test')
    ground_truth = np.stack([simulation_data['positions_x'], simulation_data['positions_y']], axis=1)
    zs = np.stack([simulation_data['accelerometer_x'], simulation_data['accelerometer_y'], simulation_data['orientations']], axis=1)
    us = np.stack([simulation_data['acc_x'], simulation_data['acc_y'], simulation_data['steering'].values], axis=1)
    
    Ts=simulation_data['timestamps'].values
    xs = []
    L = 1.8
    N = 10000

    x_min = dm.road_points[:,0].min()
    x_max = dm.road_points[:,0].max()

    y_min = dm.road_points[:,1].min()
    y_max = dm.road_points[:,1].max()

    x_range = [x_min, x_max]
    y_range = [y_min, y_max]
    x_dot_range = [0, 10]
    y_dot_range = [0, 10]
    x_ddot_range = [0, 10]
    y_ddot_range = [0, 10]

    theta_range = [0,2*np.pi]
    delta_range = [-np.pi/2, np.pi/2]
    particles = create_uniform_particles(x_range, y_range, x_dot_range, y_dot_range,x_ddot_range, y_ddot_range, theta_range, delta_range, N)
    weights = np.full((particles.shape[0],), 1/particles.shape[0])
    std = np.array([0.2, 0.2, 0.2])

  
    for i,u in enumerate(Ts): 

        predict(particles, us[i], std, Ts[i] - Ts[i-1], L)
        
        # add noise to measurement (later done in carla?)
        zs[i] += (np.random.randn(len(zs[i]))*sensor_std)

        update(particles, weights, zs[i], 0, dm)
        
        if (neff(weights) < N/2): 
            print("resample")
            indexes = systematic_resample(weights)
            resample_from_index(particles, weights, indexes)
            assert np.allclose(weights, 1/N)

        if (i % 100 == 0): 
            print(i, " iterations done")
        mu, var = estimate(particles,weights)
        
        xs.append(mu)
    xs = np.array(xs)
    return particles, weights, xs, ground_truth
def plot_result(xs, ground_truth):
    xs_image_coord = []
    for x in xs: 
        xs_image_coord.append(dm.coord_to_image(x))
    xs_image_coord = np.array(xs_image_coord)

    gt_image_coord = []
    for gt in ground_truth: 
        gt_image_coord.append(dm.coord_to_image(gt))
    gt_image_coord = np.array(gt_image_coord)

    plt.imshow(dm.distance_map, cmap="gray")
    plt.scatter(xs_image_coord[:,0], xs_image_coord[:,1], label="estimations", alpha = 0.1)
    plt.scatter(gt_image_coord[:,0],gt_image_coord[:,1], label="ground truth")
    #plt.ylim([2500, 0])
    #plt.xlim([0, 2500])
    plt.legend()
    plt.show()

def main(): 
    simulation_data = prepare_data()
    particles, weights, xs, ground_truth = run_pf_imu(simulation_data=simulation_data, sensor_std=2, std=np.array([0.02, 0.02, 0.02]))
    plot_result(xs, ground_truth)

if __name__ == '__main__':

    main()
