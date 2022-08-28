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
import utils
import config


'''
Update calculates the weights for each particle with the acceleration measurement, the orientation measurement and the distance to a road
'''
def update(particles, weights,z, R, dm):
    
    acc_measured = np.linalg.norm(z[0:2])
    # acceleration likelihood
    acceleration_likelihoods = stats.norm(particles[:,3], (R[0]+R[1])/2).pdf(acc_measured)
    # orientation likelihood
    rotation_likelihoods = stats.norm(particles[:,4], R[2]).pdf(z[2])

    '''
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
            value = dm.distance_map[p[1], p[0]]
            if (value <= 0.7): 
                value /= 50
            else: 
                value = 1
            distances.append(value)
        else:
            distances.append(0)
    
    distances = np.array(distances, dtype=object)
    average = np.array((acceleration_likelihoods + rotation_likelihoods + distances) / 4)
    #weights = weights * acceleration_likelihoods * rotation_likelihoods
    weights =  weights * (distances) 
    #old approach
    #weights = distances

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

# x = x, y, v, a, theta, delta


# u = acc, steering 
# z = acc_x, acc_y, rot
'''
Transfers the state into the next 
'''
def F(x, u, step, L, std, N): 
    # calculate nect velocity
    v_next = x[2] + (x[3] * step)
    # control input as next acceleration
    a_next = u[0] + (np.random.uniform(0,1,1) * std[0])
    # get next position
    x_next = x[0] + (x[2]*np.cos(x[4] + x[5]) * step)
    y_next = x[1] + (x[2]*np.sin(x[4] + x[5]) * step)
    #calculate theta and delta
    theta_next = x[4] + (((x[2]*np.sin(x[5]))/L) * step)
    delta_next = x[5] + ((u[1] + np.random.uniform(0,1,1)*std[1]) * step)
    return np.array([x_next, y_next,v_next, a_next, theta_next, delta_next], dtype=object) 


def F_old(x, u, step, L, std, N): 
    '''This function uses an acceleration and velocity with a directon. The two wheeled bycicle model doesnt need that '''
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
    '''
    Calls F for every particle
    '''
    N = len(particles) 
    # Needs noise: not in a for loop
    for i in range(N): 
        particles[i] = F(x=particles[i], u=u, step=dt, L=L, std=std, N=len(particles))

'''
creates uniformly distributed particles
'''
def create_uniform_particles(x_range, y_range, v_range, a_range, theta_range, delta_range,N):
    particles = np.empty((N, 6))
    particles[:, 0] = uniform(x_range[0], x_range[1], size=N)
    particles[:, 1] = uniform(y_range[0], y_range[1], size=N)
    particles[:, 2] = uniform(v_range[0], v_range[1], size=N)
    particles[:, 3] = uniform(a_range[0], a_range[1], size=N)
    particles[:, 4] = uniform(theta_range[0], theta_range[1], size=N)
    particles[:, 5] = uniform(delta_range[0], delta_range[1], size=N)
    particles[:, 4] %= 2 * np.pi
    particles[:, 5] %= 2 * np.pi
    return particles



'''
main function running the pf
'''
def run_pf_imu(simulation_data, sensor_std, std,dm):
   
    ground_truth = np.stack([simulation_data['positions_x'], simulation_data['positions_y']], axis=1)
    zs = np.stack([simulation_data['acc_x_noise'], simulation_data['acc_y_noise'], simulation_data['orientations']], axis=1)
    us = np.stack([simulation_data['acceleration_control_input'], simulation_data['steering'].values], axis=1)
    
    Ts=simulation_data['timestamps'].values
    xs = []
    dt=1/10
    particles_at_t = []
    weights_at_t = []
    ground_truth_at_t = []

    L = config.L
    N = config.N

    x_min = dm.road_points[:,0].min()
    x_max = dm.road_points[:,0].max()
    
    y_min = dm.road_points[:,1].min()
    y_max = dm.road_points[:,1].max()
    
    
 
    x_range = [x_min, x_max]
    y_range = [y_min, y_max]
   
    v_range = [0, 10]
    a_range = [0, 10]

    theta_range = [0,2*np.pi]
    delta_range = [-np.pi/2, np.pi/2]
    particles = create_uniform_particles(x_range, y_range, v_range, a_range, theta_range, delta_range, N)
 
    weights = np.full((particles.shape[0],), 1/particles.shape[0])
    std = np.array([0.2, 0.02])

    particle_values = []
    measurement_values = []
  
    for i,u in enumerate(Ts): 
        # Test plot
        particles_at_t.append(copy.copy(particles))
        weights_at_t.append(copy.copy(weights))
        ground_truth_at_t.append(copy.copy(ground_truth[i]))

        predict(particles=particles, u=us[i], std=std, dt=dt, L=L)

        # add noise to measurement (later done in carla?) --> We create the noise in dara preperation
        #zs[i] += (np.random.randn(len(zs[i]))*sensor_std)

        update(particles=particles, weights=weights, z=zs[i], R=sensor_std, dm=dm)
        
        if (i == 100): 
            measurement_values = np.full((N,3 ), zs[i])   

            particle_values = np.stack([particles[:,3], particles[:,4], measurement_values[:,0], measurement_values[:,1], measurement_values[:,2]], axis=1)
                     
            utils.write_to_csv("test_likelihood", particle_values )


        if (neff(weights) < N/4): 
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
        ax.scatter(particles_image[:,0], particles_image[:,1], color="b", label="particles", s = weights[i] * 100)
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
    simulation_data = data_preperation.prepare_data(config.simulation_data_dataset)
    dm = distance_map.DistanceMap(1, 300, config.road_points_dataset)

    particles, weights, xs, ground_truth, Ts = run_pf_imu(simulation_data=simulation_data, sensor_std=config.sensor_std, std=config.std,dm=dm)
    #plot_result(particles, xs, ground_truth, dm)
    plot_results_animated(particles=particles, weights=weights, xs=xs, ground_truth=ground_truth, dm=dm, Ts=Ts)
if __name__ == '__main__':

    main()
