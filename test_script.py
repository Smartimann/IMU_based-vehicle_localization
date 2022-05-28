import matplotlib.pyplot as plt
import numpy as np
from parse_map import plot_map


import parse_map as pm
import check_position as cp
import utils

def main(): 
    lines, arcs, environment = utils.load_map('road_points_data', 'surroundings')
    position_check = cp.PositionCheck(lines, arcs, 1)
    plot_map(lines, arcs, environment)
    is_on_road, lin_dist, arc_dist = position_check.is_on_road(np.array([200,200]))
    print(is_on_road)
    print(lin_dist)
    print(arc_dist)

if __name__ == '__main__':

    main()