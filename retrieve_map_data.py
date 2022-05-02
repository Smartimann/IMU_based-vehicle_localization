import sys
import os
try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla')
except IndexError:
    pass

import carla

def get_simulation_map(args, client): 
    world = client.get_world()
    map = world.get_map()
    map.save_to_disk('data')