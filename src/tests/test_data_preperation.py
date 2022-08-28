import numpy as np
import sys
import os
PROJECT_ROOT = os.path.abspath(os.path.join(
                  os.path.dirname('nav_filter'), 
                  os.pardir)
)
sys.path.append(PROJECT_ROOT)
print(PROJECT_ROOT)
from nav_filter.filterscripts import data_preperation as dp



def test_get_rotation_matrix(): 
    theta = np.deg2rad(90)
    rot_matrix = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    assert np.array_equal(dp.get_rotation_matrix(theta), rot_matrix)

def test_linear_coord_transformation_with_acc_in_x_direction(): 
    theta = np.deg2rad(90)
    local_acc = np.array([1,0])
    rot_matrix = dp.get_rotation_matrix(theta)

    expected_global_acc = np.array([0,1])
    calculated_global_acc = np.dot(rot_matrix, local_acc)
    print(local_acc- calculated_global_acc)
    is_close_enough = expected_global_acc[0] - calculated_global_acc[0] < 0.0000001 and expected_global_acc[1] - calculated_global_acc[1] < 0.0000001
    
    assert  is_close_enough


def test_linear_coord_transformation_with_acc_in_x_and_y_direction(): 
    theta = np.deg2rad(45)
    local_acc = np.array([1,1])
    rot_matrix = dp.get_rotation_matrix(theta)

    expected_global_acc = np.array([1,0])
    calculated_global_acc = np.dot(local_acc, rot_matrix)
    print(local_acc- calculated_global_acc)
    is_close_enough = expected_global_acc[0] - calculated_global_acc[0] < 0.4 and expected_global_acc[1] - calculated_global_acc[1] < 0.0000001
    
    assert  is_close_enough
