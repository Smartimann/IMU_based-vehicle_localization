import pandas as pd 
import numpy as np 
import matplotlib.pyplot as plt
import sys
import os
from numba import jit,njit, vectorize, cuda
import math
import cv2 as cv

from pip import main
PROJECT_ROOT = os.path.abspath(os.path.join(
                  os.path.dirname('utils'), 
                  os.pardir)
)
sys.path.append(PROJECT_ROOT)
import utils

class DistanceMap(): 
    def __init__(self, decimal_places, additional_border, filename) -> None:
        pass
        self.decimal_places = decimal_places
        self.additional_border = additional_border
        self.to_array_indices = 10**decimal_places
        self.to_coordinates = 10**-decimal_places
        self.lines, self.arcs= utils.load_map(filename)
        self.road_points = np.concatenate([self.lines, self.arcs])
        self.road_point_indicies = (np.round(self.road_points,decimal_places)*self.to_array_indices).astype(int)
        self.x_min_bound = self.road_point_indicies[:,0].min() - additional_border
        self.y_min_bound = self.road_point_indicies[:,1].min() - additional_border
        self.x_max_bound = self.road_point_indicies[:,0].max() + additional_border
        self.y_max_bound = self.road_point_indicies[:,1].max() + additional_border
        self.size_x = int(self.x_max_bound - self.x_min_bound)
        self.size_y = int(self.y_max_bound - self.y_min_bound)
        self.map_array = np.zeros((self.y_max_bound + (0 - self.y_min_bound), self.x_max_bound + (0 -self.x_min_bound)))
        #self.map_array = self.map_array[200:300, 200:300]
        self.converted_points = []
        self.distance_map = np.zeros_like(self.map_array)
        self.create()


    def create(self): 
        self.fill_map_with_road_points()
        self.create_distance_map()

    def create_distance_map(self): 
        self.map_array = cv.cvtColor()
        self.distance_map = cv.distanceTransform(self.map_array, cv.DIST_L2, 3)

    def convert_coord_into_image(self, point):
        '''Converts a coordinate from map coodrinates to image coordinates'''
        return point + np.array([0-(self.x_min_bound ), 0-(self.y_min_bound)])

    def convert_image_into_coord(self, point): 
        '''Converts a coordinate from ingame coodrinates to map coordinates'''
        return point - np.array([0-(self.x_min_bound ), 0-(self.y_min_bound)])

    def fill_map_with_road_points(self):    
        for p in self.road_point_indicies: 
            converted_point = self.convert_coord_into_image(p)
            self.map_array[converted_point[1], converted_point[0]] = 1
            self.converted_points.append(converted_point)
        self.converted_points = np.array(self.converted_points)
        print("Converted Points", self.converted_points.shape)
        print(self.map_array.shape)
        #self.map_array = self.map_array[200:300, 200:300]

    def show_map(self): 
        print("Show map")
        plt.imshow(self.map_array, cmap='gray')
        #plt.imsave('map',self.map_array)
        plt.show()
    def show_distance_map(self, name): 
        #plt.imsave(PROJECT_ROOT+'\\'+name+'.png', self.distance_map, cmap='gray', format="png")
        plt.imshow(self.distance_map, cmap='gray')
        plt.show()



def main(): 
    distance_map = DistanceMap(1, 100, 'road_points_data_test')
    #distance_map.show_map()
    print("Finished creating map array")
    distance_map.show_distance_map("manno")
    print("Finished creating distance map")
    #distance_map.show_distance_map(str(distance_map.decimal_places)+'_'+str(distance_map.additional_border))
    #plt.imsave(PROJECT_ROOT+'\\map.png', np.random.rand(100,100), cmap='gray', format="png")
if __name__ == '__main__':
    main()    



