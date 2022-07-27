import pandas as pd 
import numpy as np 
import matplotlib.pyplot as plt
import sys
import os
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
        self.map_array = np.zeros((self.y_max_bound + (0 - self.y_min_bound), self.x_max_bound + (0 -self.x_min_bound)), np.uint8)
        self.converted_points = []
        self.distance_map = np.zeros_like(self.map_array)
        self.create()


    def create(self): 
        self.fill_map_with_road_points()
        self.create_distance_map()

    def create_distance_map(self): 
        inverted_map = 1-self.map_array
        self.distance_map = cv.distanceTransform(inverted_map, cv.DIST_L2, 3, cv.CV_8U)
        cv.normalize(self.distance_map, self.distance_map, 0, 1.0, cv.NORM_MINMAX)
        self.distance_map = 1- self.distance_map

    def coord_to_image(self, point): 
        point = (point * self.to_array_indices).astype(int)
        point = self.add_bounds_to_match_image_frame(point)
        return point

    def image_to_coord(self, point): 
        point = point * self.to_coordinates
        point = self.remove_bounds_to_match_coord_frame(point)
        return point

    def add_bounds_to_match_image_frame(self, point):
        '''Converts a coordinate from map coodrinates to image coordinates'''
        return point + np.array([0-(self.x_min_bound ), 0-(self.y_min_bound)])

    def remove_bounds_to_match_coord_frame(self, point): 
        '''Converts a coordinate from ingame coodrinates to map coordinates'''
        return point - np.array([0-(self.x_min_bound ), 0-(self.y_min_bound)])

    def fill_map_with_road_points(self):    
        for p in self.road_point_indicies: 
            converted_point = self.add_bounds_to_match_image_frame(p)
            #converted_point = p
            self.map_array[converted_point[1], converted_point[0]] = 1
            self.converted_points.append(converted_point)
        self.converted_points = np.array(self.converted_points)
        print("Converted Points", self.converted_points.shape)
        print(self.map_array.shape)
        #self.map_array = self.map_array[200:300, 200:300]

    def save_map(self): 
        print("Show map")
        
        cv.imwrite(PROJECT_ROOT + "\..\data\images\map.png",255*self.map_array)

    def save_distance_map(self): 
        cv.imwrite(PROJECT_ROOT + "\..\data\images\distanceMap.png",255*self.distance_map)
                          



def main(): 
    distance_map = DistanceMap(1, 500, 'road_points_data')
    distance_map.save_map()
    print("Finished creating map array")
    distance_map.save_distance_map()
    print("Finished creating distance map")
    #distance_map.save_distance_map(str(distance_map.decimal_places)+'_'+str(distance_map.additional_border))
    #plt.imsave(PROJECT_ROOT+'\\map.png', np.random.rand(100,100), cmap='gray', format="png")
if __name__ == '__main__':
    main()    



