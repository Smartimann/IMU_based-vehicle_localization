import numpy as np
import pandas as pd
import sys 
import os

PROJECT_ROOT = os.path.abspath(os.path.join(
                  os.path.dirname(__file__), 
                  os.pardir)
)
sys.path.append(PROJECT_ROOT)
def load_map(roads_filename): 
    path_roads = PROJECT_ROOT +'\\..\\data\\'+str(roads_filename)+'.csv'
    df_roads = pd.read_csv(path_roads)
    line_points_x = df_roads.loc[df_roads['Type'] == 'line']['X'].values
    line_points_y = df_roads.loc[df_roads['Type'] == 'line']['Y'].values
    line_points = np.stack([line_points_x, line_points_y], axis=-1)

    arc_points_x = df_roads.loc[df_roads['Type'] == 'arc']['X'].values
    arc_points_y = df_roads.loc[df_roads['Type'] == 'arc']['Y'].values
    arc_points = np.stack([arc_points_x, arc_points_y], axis=-1)
    '''
    path_env = 'data/'+str(env_filename) + '.csv'
    df_env = pd.read_csv(path_env)

    bound_trans_x = df_env['Transform_loc_x']
    bound_trans_y = df_env['Transform_loc_y']
    bound_boxes_x = df_env['Bounding_loc_x']
    bound_boxes_y = df_env['Bounding_loc_y']
    bound_boxes_extent_x = df_env['Bounding_extent_x']
    bound_boxes_extent_y = df_env['Bounding_extent_y']

    bounding_boxes = np.stack([bound_trans_x, bound_trans_y ,bound_boxes_x, bound_boxes_y, bound_boxes_extent_x, bound_boxes_extent_y],axis=-1)
    '''
    return line_points, arc_points