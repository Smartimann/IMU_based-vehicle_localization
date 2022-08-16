import numpy as np
import sys
import os
PROJECT_ROOT = os.path.abspath(os.path.join(
                  os.path.dirname('nav_filter'), 
                  os.pardir)
)
sys.path.append(PROJECT_ROOT)
print(PROJECT_ROOT)
from nav_filter.filterscripts import calculate_differences
from nav_filter.filterscripts import calculate_weights

import math

'''
This tests test the function get_rotation_difference
'''
def test_get_rotation_differences_0_0():
    angle_one = np.deg2rad(0)
    angle_two = np.deg2rad(0)
    rot_diff = calculate_differences.get_rotation_difference(angle_one, angle_two)
    print("Result: ", angle_one, angle_two, rot_diff)
    assert rot_diff == np.deg2rad(0)

def test_get_rotation_differences_0_90(): 
    angle_one = np.deg2rad(0)
    angle_two = np.deg2rad(90)
    rot_diff = calculate_differences.get_rotation_difference(angle_one, angle_two)
    assert rot_diff == 1.414213562373095

def test_get_rotation_differences_5_360(): 
    angle_one = np.deg2rad(5)
    angle_two = np.deg2rad(360)
    rot_diff = calculate_differences.get_rotation_difference(angle_one, angle_two)
    print("Result: ", angle_one, angle_two, rot_diff)
    assert rot_diff == 0.08723877473067224



def test_get_rotation_differences_0_270(): 
    angle_one = np.deg2rad(0)
    angle_two = np.deg2rad(270)
    rot_diff = calculate_differences.get_rotation_difference(angle_one, angle_two)
    print("Result: ", angle_one, angle_two, rot_diff)
    assert rot_diff == 1.4142135623730951


def test_get_rotation_differences_0_360(): 
    angle_one = np.deg2rad(0)
    angle_two = np.deg2rad(360)
    rot_diff = calculate_differences.get_rotation_difference(angle_one, angle_two)
    print("Result: ", angle_one, angle_two, rot_diff)
    assert rot_diff <= 0.000001


def test_rotation_difference_for_np_array_length(): 
    test_array = np.full((10,4), np.pi)
    test_angle = np.pi

    test_angle_array = np.full((10,), test_angle)
    result = map(calculate_differences.get_rotation_difference,test_array[:,1],test_angle_array)
    
    assert len(list(result)) == 10

def test_rotation_difference_for_np_array_values(): 
    test_array = np.array([0,np.deg2rad(360), np.deg2rad(180), np.deg2rad(350)])
    test_array_2 = np.full(len(test_array), 0)
    result = list(map(calculate_differences.get_rotation_difference,test_array,test_array_2))
    print(np.rad2deg(result))
    zero_right = result[0] == 0
    one_right = math.isclose(result[1], 0, abs_tol=1e-8)
 
    two_right = math.isclose(result[2], 2.0, abs_tol=1e-8)
    print(result[2])
    three_right = math.isclose(result[3],0.17431148549531633, abs_tol=1e-8)

    assert zero_right and one_right and two_right and three_right
    

'''
These tests test the function get_acceleration_difference
'''
def test_acceleration_difference_same_acc(): 
    acc_a = np.array([1,0])
    acc_b = np.array([1,0])

    acc_diff = calculate_differences.get_acceleration_difference(acc_a, acc_b)
    assert acc_diff == 0


def test_acceleration_difference_same_x(): 
    acc_a = np.array([3,4])
    acc_b = np.array([3,1])

    acc_diff = calculate_differences.get_acceleration_difference(acc_a, acc_b)
    assert acc_diff == 3

def test_acceleration_difference_for_map_array_length(): 
    test_array = np.array([
        np.array([1,0]),
        np.array([0,1]),
        np.array([1,1])
    ])

    test_array_2 = np.array([
        np.array([0,2]),
        np.array([1,1]),
        np.array([3,1])
    ])

    result = list(map(calculate_differences.get_acceleration_difference, test_array, test_array_2))
    assert len(result) == 3

'''
These tests test the calculation of the weights based on differences
'''

def test_calculate_weights_from_differences(): 
    test_array = np.array([-1,20, 5,4])
    result = calculate_weights.calculate_weights_from_differences(test_array)

    assert np.allclose(result,np.array([1.        , 0.        , 0.71428571, 0.76190476]),rtol=0.00000001)

def test_get_weight_mean(): 
    rot_weights = np.array([1,2,3])
    acc_weights = np.array([2,3,4])
    dm_weights = np.array([3,4,5])

    result = calculate_weights.get_weight_mean(rot_weights, acc_weights, dm_weights)
    
    assert np.allclose(result, np.array([2, 3, 4]),rtol=0.00000001)

