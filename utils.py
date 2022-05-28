import numpy as np 
import pandas as pd

import math

def write_to_csv(name, data): 
    path = 'data/'+str(name)+'.csv'
    df = pd.DataFrame(data)

    df.to_csv(path)


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
    write_to_csv('road_points_'+name, data_dict)


def load_map(roads_filename, env_filename): 
    path_roads = 'data/'+str(roads_filename)+'.csv'
    df_roads = pd.read_csv(path_roads)
    line_points_x = df_roads.loc[df_roads['Type'] == 'line']['X'].values
    line_points_y = df_roads.loc[df_roads['Type'] == 'line']['Y'].values
    line_points = np.stack([line_points_x, line_points_y], axis=-1)

    arc_points_x = df_roads.loc[df_roads['Type'] == 'arc']['X'].values
    arc_points_y = df_roads.loc[df_roads['Type'] == 'arc']['Y'].values
    arc_points = np.stack([arc_points_x, arc_points_y], axis=-1)

    path_env = 'data/'+str(env_filename) + '.csv'
    df_env = pd.read_csv(path_env)

    bound_trans_x = df_env['Transform_loc_x']
    bound_trans_y = df_env['Transform_loc_y']
    bound_boxes_x = df_env['Bounding_loc_x']
    bound_boxes_y = df_env['Bounding_loc_y']
    bound_boxes_extent_x = df_env['Bounding_extent_x']
    bound_boxes_extent_y = df_env['Bounding_extent_y']

    bounding_boxes = np.stack([bound_trans_x, bound_trans_y ,bound_boxes_x, bound_boxes_y, bound_boxes_extent_x, bound_boxes_extent_y],axis=-1)

    return line_points, arc_points, bounding_boxes

    