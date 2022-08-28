#!/usr/bin/env python

# Copyright (c) 2018 Intel Labs.
# authors: German Ros (german.ros@intel.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Example of automatic vehicle control from client side."""

from __future__ import print_function

import os
from re import S
import sys
from typing import Iterator
# Define root folder
PROJECT_ROOT = os.path.abspath(os.path.join(
                  os.path.dirname(__file__), 
                  os.pardir)
)
sys.path.append(PROJECT_ROOT)

import argparse
import datetime
import glob
from time import time
import numpy.random as random
import utils

from queue import Queue
from queue import Empty

try:
    import numpy as np
except ImportError:
    raise RuntimeError(
        'cannot import numpy, make sure numpy package is installed')


# Find CARLA module 
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# Add PythonAPI for release mode 
try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla')
except IndexError:
    pass

import carla


def run_simulation(args, client): 
    actor_list = []
    accelerometer_values = []
    accelerometer_values_noise = []
    timestamps = []
    positions = []
    orientations = []
    steerings = []
    throttles = []
    try: 
        # Get the world and apply world settings
        world = client.get_world()
        settings = world.get_settings()
        if not settings.synchronous_mode:
            settings.synchronous_mode = True
        # fixed_delta_seconds need  <= max_substep_delta_time * max_substeps
        settings.fixed_delta_seconds = .05
        settings.max_substep_delta_time = 0.01
        settings.max_substeps = 10
        print("Settings", settings)
        world.apply_settings(settings)

        blueprint_library = world.get_blueprint_library()

        bp_vehicle = blueprint_library.filter('a2')[0]


        if bp_vehicle.has_attribute('color'): 
            color = random.choice(bp_vehicle.get_attribute('color').recommended_values)
            bp_vehicle.set_attribute('color', color)
        
        transform = random.choice(world.get_map().get_spawn_points())
        vehicle = world.spawn_actor(bp_vehicle, transform)

        actor_list.append(vehicle)
        print('created %s' % vehicle.type_id)
        
        tm = client.get_trafficmanager()
        tm_port = tm.get_port()
        tm.set_synchronous_mode(True)
        #vehicle.set_autopilot(True)
        vehicle.set_autopilot(True, tm_port)
        vehicle_control = vehicle.get_control()
        
        #spectator = world.get_spectator()
        #spectator.set_transform(vehicle.get_transform())


        # --------------
        # Add IMU sensor to ego vehicle. 
        # --------------
        imu_bp = world.get_blueprint_library().find('sensor.other.imu')
        imu_location = carla.Location(0,0,0)
        imu_rotation = carla.Rotation(0,0,0)
        imu_transform = carla.Transform(imu_location,imu_rotation)
        imu_bp.set_attribute("sensor_tick",str(1/10))
        ego_imu = world.spawn_actor(imu_bp,imu_transform,attach_to=vehicle, attachment_type=carla.AttachmentType.Rigid)

        def imu_callback(imu):
            accelerometer_values.append(np.array([imu.accelerometer.x, imu.accelerometer.y, imu.accelerometer.z]))  
            print(imu.accelerometer)    
            timestamps.append(imu.timestamp)
            steerings.append(vehicle.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel))
            throttles.append(vehicle_control.throttle)
            orientations.append((imu.compass + np.pi) % np.pi*2)
            positions.append(np.array([vehicle.get_transform().location.x, vehicle.get_transform().location.y]))
            

        ego_imu.listen(lambda imu: imu_callback(imu))
        # game loop
        while True: 
            if args.sync:
                world.tick()
            else:
                world.wait_for_tick()
    finally: 
        print("End sensor retrievment")
        ego_imu.destroy()
        #client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        vehicle.destroy()
        print("Finished retrieving sensor data")
        accelerometer_values = np.array(accelerometer_values)
        accelerometer_values_noise = np.array(accelerometer_values_noise)
        timestamps = np.array(timestamps)
        positions = np.array(positions)
        

        retrieved_data = {
            'accelerometer_x': accelerometer_values[:, 0], 
            'accelerometer_y': accelerometer_values[:, 1], 
            'accelerometer_z': accelerometer_values[:, 2],
            'orientations': orientations,
            'steering': steerings,
            'throttle': throttles,
            'positions_x': positions[:,0],
            'positions_y': positions[:,1],
            'timestamps': timestamps
        }
        today = datetime.datetime.now().strftime("%d_%m_%y_%H_%M_%S")
        utils.write_to_csv("Simulation_test_run"+str(today), retrieved_data)
     

        positions = np.array(positions)
        position_data = {
            'x': positions[:,0], 
            'y': positions[:,1], 
            'timestamps': timestamps
        }
        utils.write_to_csv('car_positions', position_data)
        print('destorying all actors')

        print('done.')

def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Sensor tutorial')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Synchronous mode execution')
    argparser.add_argument(
        '--async',
        dest='sync',
        action='store_false',
        help='Asynchronous mode execution')
    argparser.set_defaults(sync=True)
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720)')

    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(5.0)

        run_simulation(args, client)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')    
if __name__ == '__main__':

    main()
