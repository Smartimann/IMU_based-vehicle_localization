import numpy as np

class PositionCheck: 
    def __init__(self, line_points, arc_points, road_distance):
        self.line_points = line_points 
        self.arc_points = arc_points
        self.road_distance = road_distance

    def is_on_road(self, position: np.ndarray): 
        distances_to_line_points = [np.linalg.norm(position, line_point) for line_point in self.line_points]
        distances_to_arc_points = [np.linalg.norm(position, arc_point) for arc_point in self.arc_points]

        points_on_road_line = []
        points_on_road_arc = np.where(distances_to_arc_points < self.road_distance)
        
        if (len(points_on_road_line) > 0 or len(points_on_road_arc) > 0): 
            return True, distances_to_line_points, distances_to_arc_points 
        else: 
            return False, distances_to_line_points, distances_to_arc_points

    def distance_between_points(self, point_a, point_b):
        distance = np.sqrt((point_a[0]-point_b[0])**2 + (point_a[1]-point_b[1]**2)) 
        return distance