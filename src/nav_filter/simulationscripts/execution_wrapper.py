
from __future__ import print_function

import argparse
import os
import numpy.random as random
import sys

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
        '-sn','--script_name', 
        dest='script_name',
        metavar="SD",
        help='choose: get_sensor_data, retrieve_map_data, change_weather, get_surroundings'
    )

    argparser.add_argument(
        '-dt', '--daytime', 
        dest='daytime',
        default='day',
        help='choose: day, night'
    )
    argparser.add_argument(
        '-w','--weather',
        default='clear',
        dest='weather',
        help='choose: clear, light_fog, heavy_fog, light_rain, heavy_rain, rainy_storm, clear_storm'
    )

  



    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)

        # If a connection to the server could be established, import the wanted script and 
        # run its code
        if (client): 
            print("connection worked")
            if (args.script_name == 'get_sensor_data'): 
                import get_sensor_data as gsd
                gsd.run_simulation(args, client)
            elif (args.script_name == 'retrieve_map_data'): 
                import retrieve_map_data as rmd
                rmd.get_simulation_map(args, client)
            elif(args.script_name == 'change_weather'): 
                import change_weather as cw
                cw.change_weather(args, client)
            elif(args.script_name == 'get_surroundings'): 
                import get_surroundings as gs
                gs.get_surroundings(args, client)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')    
if __name__ == '__main__':

    main()
