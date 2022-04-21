
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
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla')
except IndexError:
    pass

import carla
# scripts that can be executed

def main():
    argparser = argparse.ArgumentParser(
        description='Execution wrapper for carla scripts')
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
    argparser.add_argument(
        '--script_name', 
        metavar="SD",
        default='get_sensor_data', 
        help='specifiy the scripts name you want to execute'
    )

    argparser.add_argument()

    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(5.0)

        # If a connection to the server could be established, import the wanted script and 
        # run its code
        if (client): 
            print("connection worked")
            if (args.script_name == 'get_sensor_data'): 
                import get_sensor_data
                get_sensor_data.run_simulation(args, client)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')    
if __name__ == '__main__':

    main()
