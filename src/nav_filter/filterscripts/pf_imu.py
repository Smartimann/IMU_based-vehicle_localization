import sys
import os
PROJECT_ROOT = os.path.abspath(os.path.join(
                  os.path.dirname('nav_filter'), 
                  os.pardir)
)
from timeit import repeat
import numpy as np
import pandas as pd
import cv2 as cv
import matplotlib.pyplot as plt

from filterpy.monte_carlo import systematic_resample
import copy
from scipy import stats
from numpy.random import uniform
import calculate_differences
import calculate_weights
import data_preperation

from matplotlib.animation import FuncAnimation


sys.path.append(PROJECT_ROOT)
import distance_map

'''
Update calculates the weights for each particle with the acceleration measurement, the orientation measurement and the distance to a road
'''
def update(particles, weights,z, R, dm):
    

    ''' old version
    # use circular mean
    rotation_differences = np.array(list(map(calculate_differences.get_rotation_difference, particles[:,6], np.full((len(particles),),z[2]))), dtype=object)
    # content of the function abovoe:  abs(np.exp(1j*particles[:,6]/180*np.pi) - np.exp(1j*z[2]/180*np.pi))
    #rotation_differences = abs((abs(particles[:,6]-z[2])+180) %360 - 180)# index 6 = theta
    #rot_diff_weight = calculate_weights.calculate_weights_from_differences(rotation_differences)
    acceleration_difference = np.array(list(map(calculate_differences.get_acceleration_difference, np.full((len(particles),2),np.array(z[0:2])), particles[:,4:5])), dtype=object)
    #acc_diff_weight = calculate_weights.calculate_weights_from_differences(acceleration_difference)
    print(acceleration_difference.max())
    '''


    particles_image_coords = dm.coord_to_image(particles[:, 0:2])
    distances = []
    
    for p in particles_image_coords: 
        if (p[0] < dm.distance_map.shape[1] and p[0] > 0 and p[1] < dm.distance_map.shape[0] and p[1] > 0): 
            distances.append(dm.distance_map[p[1], p[0]])
        else:
            distances.append(0)

    distances = np.array(distances, dtype=object)
    #average_distances = calculate_weights.get_weight_mean(rot_diff_weight, acc_diff_weight, distances)
    

    #old approach
    weights = distances

    weights += 1.e-300
    
    weights /= sum(weights) # normalize
    
    


'''
Estimate creates an estimation of all particles
'''
def estimate(particles, weights): 
    pos = particles[:,0:2]
    mean = np.average(pos, weights=weights, axis=0)
    var  = np.average((pos - mean)**2, weights=weights, axis=0)
    return mean, var

'''
Resample function resamples all particles
'''
def resample_from_index(particles, weights, indexes):
    particles[:] = particles[indexes]
    weights.resize(len(particles))
    weights.fill (1.0 / len(weights))
    
'''
Measures the number of particles contributing to the propability distribution
'''
def neff(weights):
    return 1. / np.sum(np.square(weights))

# x = x-0,y-1,x_dot-2, y_dot-3, x_ddot-4, y_ddot-5, theta-6, delta-7
# u = acc_x, acc_y, steering 
'''
Transfers the state into the next 
'''
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
'''
Calls F for every particle
'''
def predict(particles, u, std, dt, L): 
    N = len(particles) 
    # Needs noise: not in a for loop
    for i in range(len(particles)): 
        particles[i] = F(x=particles[i], u=u, step=dt, L=L, std=std, N=len(particles))

'''
creates uniformly distributed particles
'''
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



'''
main function running the pf
'''
def run_pf_imu(simulation_data, sensor_std, std,dm):
    simulation_data = pd.read_csv("../../data/Sim_data_long.csv")
    
    ground_truth = np.stack([simulation_data['positions_x'], simulation_data['positions_y']], axis=1)
    zs = np.stack([simulation_data['accelerometer_x'], simulation_data['accelerometer_y'], simulation_data['orientations']], axis=1)
    us = np.stack([simulation_data['acc_x'], simulation_data['acc_y'], simulation_data['steering'].values], axis=1)
    
    Ts=simulation_data['timestamps'].values
    xs = []
    dt=1/10
    particles_at_t = []
    weights_at_t = []
    ground_truth_at_t = []

    L = 1.8
    N = 100

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
        # Test plot
        particles_image = np.array(list(map(dm.coord_to_image, particles[:, 0:2])))
        '''
        plt.imshow(dm.distance_map)
        plt.scatter(particles_image[:,0], particles_image[:,1], c="b")
        plt.title("Before Predict")
        plt.show()
        '''
        particles_at_t.append(copy.copy(particles))
        weights_at_t.append(copy.copy(weights))
        ground_truth_at_t.append(copy.copy(ground_truth[i]))

        predict(particles=particles, u=us[i], std=std, dt=dt, L=L)

        '''
        plt.imshow(dm.distance_map)
        plt.scatter(particles_image[:,0], particles_image[:,1], c="b")
        plt.title("After Predict")
        plt.show()
        '''


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
    particles_at_t = np.array(particles_at_t)
    weights_at_t = np.array(weights_at_t)
    ground_truth_at_t = np.array(ground_truth_at_t)
    
    return particles_at_t, weights_at_t, xs, ground_truth_at_t, Ts


def plot_result(particles, xs, ground_truth, dm):
    xs_image_coord = []
    for x in xs: 
        xs_image_coord.append(dm.coord_to_image(x))
    xs_image_coord = np.array(xs_image_coord)

    gt_image_coord = []
    for gt in ground_truth: 
        gt_image_coord.append(dm.coord_to_image(gt))
    gt_image_coord = np.array(gt_image_coord)

    p_image_coord = []
    for p in particles: 
        p_image_coord.append(np.array(list(map(dm.coord_to_image, p[:, 0:2]))))
    p_image_coord = np.array(p_image_coord)
    plt.imshow(dm.distance_map, cmap="gray")
    plt.scatter(xs_image_coord[:,0], xs_image_coord[:,1], label="estimations", alpha = 0.1)
    plt.scatter(gt_image_coord[:,0],gt_image_coord[:,1], label="ground truth")
    plt.scatter(p_image_coord[:][:,1], p_image_coord[:][:,2], label="particles")
    #plt.ylim([2500, 0])
    #plt.xlim([0, 2500])
    plt.legend()
    plt.show()


def plot_results_animated(particles, weights, xs, ground_truth, dm, Ts): 
        
    fig, ax = plt.subplots()
    def animate(i):
        # First convert data to image coordinates
        particles_image = []
        xs_image = []
        ground_truth_image = []
        particles_image = np.array(list(map(dm.coord_to_image, particles[i][:, 0:2])))
        particles_image = np.array(particles_image)
        xs_image = dm.coord_to_image(np.array(xs[i]))

        print(weights[i].max())

        ground_truth_image = dm.coord_to_image(np.array(ground_truth[i]))            
        # than plot the data
        ax.clear()
        plt.imshow(dm.distance_map, cmap="gray")
        ax.scatter(particles_image[:,0], particles_image[:,1], color="b", label="particles", s = 1/weights[i])
        #ax.plot(weights[i][:,0], weights[i][:,1], c = "yellow")
        ax.scatter(xs_image[0], xs_image[1], color="red", label="estimation")
        ax.scatter(ground_truth_image[0], ground_truth_image[1], color="green", label="ground truth")
        #ax.set_ylim([2500, 0])
        #ax.set_xlim([0, 2500])
        plt.title("At: " + str(Ts[i]))
        plt.legend()


    ani = FuncAnimation(fig, animate, frames=len(xs), interval=1/10, repeat=True)
    plt.show()

def main(): 
    simulation_data = data_preperation.prepare_data()
    dm = distance_map.DistanceMap(1, 300, 'road_points_data_test')

    particles, weights, xs, ground_truth, Ts = run_pf_imu(simulation_data=simulation_data, sensor_std=2, std=np.array([0.2, 0.2, 0.2]),dm=dm)
    #plot_result(particles, xs, ground_truth, dm)
    plot_results_animated(particles=particles, weights=weights, xs=xs, ground_truth=ground_truth, dm=dm, Ts=Ts)
if __name__ == '__main__':

    main()
