import numpy as np
import cv2 as cv
class PositionCheck: 
    def __init__(self):
        self.distance_map = cv.imread("../../data/images/map.png", cv.IMREAD_GRAYSCALE).astype('uint8')/255

    def position_check()