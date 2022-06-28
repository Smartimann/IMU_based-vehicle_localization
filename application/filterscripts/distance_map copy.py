import pandas as pd 
import numpy as np 
import math
import matplotlib.pyplot as plt
import sys
import os
from numba import jit,njit, vectorize, cuda
import time


from pip import main
PROJECT_ROOT = os.path.abspath(os.path.join(
                  os.path.dirname('utils'), 
                  os.pardir)
)
sys.path.append(PROJECT_ROOT)
import utils

filename = '../data/road_points_data25_06_22_15_59_09'
decimal_places = 2
additional_border = 100
to_array_indices = 10**decimal_places
to_coordinates = 10**-decimal_places
lines, arcs= utils.load_map(filename)
road_points = np.concatenate([lines, arcs])

road_point_indicies = (np.round(road_points,decimal_places)*to_array_indices).astype(np.intc)
#road_point_indicies = np.random.randint(0,20,size=(100,2))
x_min_bound = road_point_indicies[:,0].min() - additional_border
y_min_bound = road_point_indicies[:,1].min() - additional_border
x_max_bound = road_point_indicies[:,0].max() + additional_border
y_max_bound = road_point_indicies[:,1].max() + additional_border
size_x = int(x_max_bound - x_min_bound)
size_y = int(y_max_bound - y_min_bound)
#distance_map = np.zeros_like(map_array)
map_array = np.zeros((y_max_bound + (0 - y_min_bound), x_max_bound + (0 -x_min_bound)), dtype=np.float64)
converted_points = np.zeros_like(road_point_indicies,dtype=np.intc)
offset_array = np.array([0-(x_min_bound ), 0-(y_min_bound)]) # needed for translation points into image coord system from world


@njit
def convert_coord_into_image(point):
    '''Converts a coordinate from map coodrinates to image coordinates'''
    return point + np.array([0-(x_min_bound ), 0-(y_min_bound)])
@njit    
def convert_image_into_coord(point): 
    '''Converts a coordinate from ingame coodrinates to map coordinates'''
    return point - np.array([0-(x_min_bound ), 0-(y_min_bound)])


def fill_map_with_road_points_slow(road_point_indicies, map_array, converted_points):    
    
    for p in road_point_indicies: 
        converted_point = convert_coord_into_image(p)
        map_array[converted_point[1], converted_point[0]] = 1
        np.append(converted_points,converted_point)
        
    print("Converted Points", converted_points.shape)
    print(map_array.shape)


@njit
def fill_map_with_road_points(road_point_indicies, map_array, converted_points):  
    for i in range(len(road_point_indicies)): 
        converted_point = convert_coord_into_image(road_point_indicies[i])
        map_array[converted_point[1],converted_point[0]] = 1
        np.append(converted_points, converted_point)


def main(): 
    start_time = time.time()
    fill_map_with_road_points_slow(np.random.randint((1,2)), map_array, converted_points)
    end_time = time.time()

    time_elapsed = (end_time - start_time)
    print(str(time_elapsed)+" s")
    print("Estimated time: " + str(time_elapsed * road_point_indicies.shape[0]/60/60))
     
if __name__ == '__main__':
    main()   