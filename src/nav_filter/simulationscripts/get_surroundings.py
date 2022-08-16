#!/usr/bin/env python

# Copyright (c) 2018 Intel Labs.
# authors: German Ros (german.ros@intel.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Example of automatic vehicle control from client side."""

from __future__ import print_function
import datetime
import glob
import os
import numpy.random as random
import sys
import utils
from matplotlib import cm
import open3d as o3d
import time




try:
    import numpy as np
except ImportError:
    raise RuntimeError(
        'cannot import numpy, make sure numpy package is installed')

# ==============================================================================
# -- Find CARLA module ---------------------------------------------------------
# ==============================================================================
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# ==============================================================================
# -- Add PythonAPI for release mode --------------------------------------------
# ==============================================================================
try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla')
except IndexError:
    pass

import carla

get_lidar = True
VIRIDIS = np.array(cm.get_cmap('plasma').colors)
VID_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])
LABEL_COLORS = np.array([
    (255, 255, 255), # None
    (70, 70, 70),    # Building
    (100, 40, 40),   # Fences
    (55, 90, 80),    # Other
    (220, 20, 60),   # Pedestrian
    (153, 153, 153), # Pole
    (157, 234, 50),  # RoadLines
    (128, 64, 128),  # Road
    (244, 35, 232),  # Sidewalk
    (107, 142, 35),  # Vegetation
    (0, 0, 142),     # Vehicle
    (102, 102, 156), # Wall
    (220, 220, 0),   # TrafficSign
    (70, 130, 180),  # Sky
    (81, 0, 81),     # Ground
    (150, 100, 100), # Bridge
    (230, 150, 140), # RailTrack
    (180, 165, 180), # GuardRail
    (250, 170, 30),  # TrafficLight
    (110, 190, 160), # Static
    (170, 120, 50),  # Dynamic
    (45, 60, 150),   # Water
    (145, 170, 100), # Terrain
]) / 255.0 # normalize each channel [0-1] since is what Open3D uses


def get_surroundings(args, client):
    if (get_lidar): 
        get_lidar_surroundings(args, client)
    else: 
        get_map_surroundings(args, client)





def generate_lidar_bp(world, blueprint_library, delta):
    """Generates a CARLA blueprint based on the script parameters"""
  
    lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
    lidar_bp.set_attribute('dropoff_general_rate', '0.0')
    lidar_bp.set_attribute('dropoff_intensity_limit', '1.0')
    lidar_bp.set_attribute('dropoff_zero_intensity', '0.0')
    
    lidar_bp.set_attribute('upper_fov', str(15.0))
    lidar_bp.set_attribute('lower_fov', str(25.0))
    lidar_bp.set_attribute('channels', str(64.0))
    lidar_bp.set_attribute('range', str(10000.0))
    lidar_bp.set_attribute('rotation_frequency', str(1.0 / delta))
    lidar_bp.set_attribute('points_per_second', str(500000))
    return lidar_bp


def add_open3d_axis(vis):
    """Add a small 3D axis on Open3D Visualizer"""
    axis = o3d.geometry.LineSet()
    axis.points = o3d.utility.Vector3dVector(np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0]]))
    axis.lines = o3d.utility.Vector2iVector(np.array([
        [0, 1],
        [0, 2],
        [0, 3]]))
    axis.colors = o3d.utility.Vector3dVector(np.array([
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0]]))
    vis.add_geometry(axis)


def get_lidar_surroundings(args, client): 

    positions = []
    client.set_timeout(10.0)
    world = client.get_world()

    try:
        original_settings = world.get_settings()
        settings = world.get_settings()
        traffic_manager = client.get_trafficmanager(8000)
        traffic_manager.set_synchronous_mode(True)

        delta = 0.05

        settings.fixed_delta_seconds = delta
        settings.synchronous_mode = True
        settings.no_rendering_mode = False
        world.apply_settings(settings)

        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter('model3')[0]
        vehicle_transform = random.choice(world.get_map().get_spawn_points())
        start_pos = np.array([vehicle_transform.location.x,vehicle_transform.location.y])
        print("Test")
        
        vehicle = world.spawn_actor(vehicle_bp, vehicle_transform)
        vehicle.set_autopilot(True)

        lidar_bp = generate_lidar_bp(world, blueprint_library, delta)

        user_offset = carla.Location(0,0,0)
        lidar_transform = carla.Transform(carla.Location(x=-0.5, z=1.8) + user_offset)

        lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=vehicle)
        point_list = o3d.geometry.PointCloud()
        def lidar_callback(point_cloud, point_list):
            """Prepares a point cloud with intensity
            colors ready to be consumed by Open3D"""
            data = np.copy(np.frombuffer(point_cloud.raw_data, dtype=np.dtype('f4')))
            data = np.reshape(data, (int(data.shape[0] / 4), 4))

   

            # Isolate the 3D data
            points = data[:, :-1]
            # We're negating the y to correclty visualize a world that matches
            # what we see in Unreal since Open3D uses a right-handed coordinate system
            points[:, :1] = -points[:, :1]
            # # An example of converting points from sensor to vehicle space if we had
            # # a carla.Transform variable named "tran":
            # points = np.append(points, np.ones((points.shape[0], 1)), axis=1)
            # points = np.dot(tran.get_matrix(), points.T).T
            # points = points[:, :-1]
            point_list.points = o3d.utility.Vector3dVector(points)
            positions.append(np.array([vehicle.get_transform().location.x, vehicle.get_transform().location.y]))


        lidar.listen(lambda data: lidar_callback(data, point_list))
        
        frame = 0
        while True:
            world.tick()
            frame += 1

    finally:
        print("End")
        #point_cloud_numpy = np.asarray(point_list.points)
        point_cloud_numpy = np.array(point_list.points)
        print(start_pos)
        point_cloud_numpy_trans =[]
        for point in point_list.points: 
            print(point[0:2])
            point_cloud_numpy_trans.append(point[0:1] - start_pos)
        point_cloud_numpy_trans = np.array(point_cloud_numpy_trans)
        #point_cloud_numpy = np.unique(point_cloud_numpy, axis=0)
        
        pc_dict = {
            'x': point_cloud_numpy[:,0],
            'y': point_cloud_numpy[:,1],
            
            
        }

        utils.write_to_csv('surroundings_pc',pc_dict)

        
        pc_dict_trans = {
            'x': point_cloud_numpy_trans[:,0],
            'y': point_cloud_numpy_trans[:,1],
            
            
        }

        utils.write_to_csv('surroundings_pc_trans',pc_dict_trans )

        new_positions = np.array(positions)
        position_data = {
            'x': new_positions[:,0], 
            'y': new_positions[:,1]
        }
        utils.write_to_csv('car_positions', position_data)
        world.apply_settings(original_settings)
        traffic_manager.set_synchronous_mode(False)
        vehicle.destroy()
        lidar.destroy()



def get_map_surroundings(args, client): 
    # Get the surroindings based on Map Data
    print("I'll get you your goddamn surroindings")
    try: 
        world = client.get_world()
        env_objs = world.get_environment_objects()
        map = world.get_map()
        print(env_objs[1000].type)

        surroundings_transforms = []
        surroundings_boundings = []
        surroundings_types = []
        #transform = map.transform_to_geolocation()
        for env_obj in env_objs: 
            if (env_obj.type != "Vegetation"): 
                surroundings_transforms.append([
                    env_obj.transform.location.x,
                    env_obj.transform.location.y, 
                    env_obj.transform.location.z, 
                    env_obj.transform.rotation.pitch, 
                    env_obj.transform.rotation.yaw, 
                    env_obj.transform.rotation.roll
                ])  
                surroundings_boundings.append([
                    env_obj.bounding_box.location.x, 
                    env_obj.bounding_box.location.y, 
                    env_obj.bounding_box.location.z,
                    env_obj.bounding_box.extent.x,
                    env_obj.bounding_box.extent.y,
                    env_obj.bounding_box.extent.z,
                    env_obj.bounding_box.rotation.pitch, 
                    env_obj.bounding_box.rotation.yaw, 
                    env_obj.bounding_box.rotation.roll
                ])
                surroundings_types.append(env_obj.type)
        surroundings_transforms = np.array(surroundings_transforms)
        surroundings_boundings = np.array(surroundings_boundings)
        surroundings_dict = {
            'Transform_loc_x': surroundings_transforms[:,0],
            'Transform_loc_y': surroundings_transforms[:,1],
            'Transform_rot_pitch': surroundings_transforms[:,3],
            'Transform_rot_yaw': surroundings_transforms[:,4],
            'Transform_rot_roll': surroundings_transforms[:,5],
            'Bounding_loc_x': surroundings_boundings[:,0],
            'Bounding_loc_y': surroundings_boundings[:,1], 
            'Bounding_extent_x': surroundings_boundings[:,3], 
            'Bounding_extent_y': surroundings_boundings[:,4],
            'Bounding_rot_pitch': surroundings_boundings[:,5],
            'Bounding_rot_yaw': surroundings_boundings[:,6],
            'Bounding_rot_roll': surroundings_boundings[:,7],  
            'Type': surroundings_types  
        }

        utils.write_to_csv('surroundings',surroundings_dict)

        # Get the surroundings based on lidar data
    finally: 
        print("Done")