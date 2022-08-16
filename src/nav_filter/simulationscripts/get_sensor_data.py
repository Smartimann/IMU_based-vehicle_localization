#!/usr/bin/env python

# Copyright (c) 2018 Intel Labs.
# authors: German Ros (german.ros@intel.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Example of automatic vehicle control from client side."""

from __future__ import print_function

import os
import sys
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
    gyroscope_values = []
    timestamps = []
    positions = []
    orientations = []
    velocities = []
    steerings = []
    throttles = []
    accelerations = [] 
    try: 
        world = client.get_world()
        blueprint_library = world.get_blueprint_library()

        bp_vehicle = blueprint_library.filter('a2')[0]

        if bp_vehicle.has_attribute('color'): 
            color = random.choice(bp_vehicle.get_attribute('color').recommended_values)
            bp_vehicle.set_attribute('color', color)
        
        transform = random.choice(world.get_map().get_spawn_points())
        vehicle = world.spawn_actor(bp_vehicle, transform)

        actor_list.append(vehicle)
        print('created %s' % vehicle.type_id)
        
        vehicle.set_autopilot(True)
        vehicle_control = vehicle.get_control()
        vehicle_physics_control = vehicle.get_physics_control()
        front_wheels = vehicle_physics_control.wheels[0:2]
        # --------------
        # Spectator on ego position
        # --------------
        spectator = world.get_spectator()
        world_snapshot = world.wait_for_tick() 
        spectator.set_transform(vehicle.get_transform())
        #measurement, sensor_data = client.read_data()
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
            gyroscope_values.append(np.array([imu.gyroscope.x, imu.gyroscope.y, imu.gyroscope.z]))
            timestamps.append(imu.timestamp)
            steerings.append(vehicle.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel))
            #print(front_wheels[0])
            throttles.append(vehicle_control.throttle)
            orientations.append((imu.compass + np.pi) % np.pi*2)
            velocities.append(np.array([vehicle.get_velocity().x, vehicle.get_velocity().y]))
            accelerations.append(np.array([vehicle.get_acceleration().x, vehicle.get_acceleration().y]))
            positions.append(np.array([vehicle.get_transform().location.x, vehicle.get_transform().location.y]))
        ego_imu.listen(lambda imu: imu_callback(imu))


        # game loop
        while True: 
            if args.sync:
                world.tick()
            else:
                world.wait_for_tick()
            
    finally: 
        accelerometer_values = np.array(accelerometer_values)
        gyroscope_values = np.array(gyroscope_values)
        timestamps = np.array(timestamps)
        velocities = np.array(velocities)
        accelerations = np.array(accelerations)
        positions = np.array(positions)
        retrieved_data = {
            'accelerometer_x': accelerometer_values[:, 0], 
            'accelerometer_y': accelerometer_values[:, 1], 
            'accelerometer_z': accelerometer_values[:, 2],
            'gyroscope_x': gyroscope_values[:,0],
            'gyroscope_y': gyroscope_values[:,1],
            'gyroscope_z': gyroscope_values[:,2], 
            'orientations': orientations,
            'velocity_x': velocities[:,0],
            'velocity_y': velocities[:,1],
            'acc_x': accelerations[:,0], 
            'acc_y': accelerations[:,1],
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
        ego_imu.destroy()
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
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
