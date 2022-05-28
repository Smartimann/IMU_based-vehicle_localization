#!/usr/bin/env python

# Copyright (c) 2018 Intel Labs.
# authors: German Ros (german.ros@intel.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Example of automatic vehicle control from client side."""

from __future__ import print_function

import argparse
import collections
import datetime
import glob
import logging
import math
import os
import numpy.random as random
import re
import sys
import weakref
import utils


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



def get_surroundings(args, client): 
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

                #print(env_obj)
        
        '''
        for top in topology: 
            locations.append([top[0].transform.location.x,top[0].transform.location.y,top[0].transform.location.z])
            locations.append([top[1].transform.location.x,top[1].transform.location.y,top[1].transform.location.z])
        
        locations = np.array(locations)
        topology_dict = {
            'X': locations[:,0], 
            'Y': locations[:,1], 
            'Z': locations[:,2]
        }

        utils.write_to_csv('topology', topology_dict)
        '''




  
    finally: 
        print("Done")
      


