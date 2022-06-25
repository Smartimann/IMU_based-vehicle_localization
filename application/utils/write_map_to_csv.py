import numpy as np 
import pandas as pd
import utils
import datetime

def write_map_to_csv(name,lines, arcs):
    '''Writes the map data as CSV to be loaded for map check'''
    line_types = np.full((len(lines),), "line")
    arc_types = np.full((len(arcs),), "arc")
    print(line_types.ndim)
    
    data_dict = {
        'X': np.concatenate([lines[:,0], arcs[:,0]]),
        'Y': np.concatenate([lines[:,1], arcs[:,1]]),
        'Type': np.concatenate([line_types[:], arc_types[:]])
    }
    today = datetime.datetime.now().strftime("%d_%m_%y_%H_%M_%S")

    utils.write_to_csv('road_points_'+name+str(today), data_dict)
