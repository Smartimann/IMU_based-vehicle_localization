import matplotlib.pyplot as plt
import numpy as np


import parse_map as pm
import check_position as cp


def main(): 
    geometries = pm.parse_map('data')
    lines, arcs = pm.calculate_raods(geometries)
    position_check = cp.PositionCheck(lines, arcs, 1)
    is_on_road, lin_dist, arc_dist = position_check.is_on_road(np.array([200,200]))
    print(is_on_road)

if __name__ == '__main__':

    main()