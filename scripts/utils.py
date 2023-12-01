##!/usr/bin/env python3
import numpy as np

def distance(pose1, pose2):
        # Define a distance function between two poses
        dx = pose1[0] - pose2[0]
        dy = pose1[1] - pose2[1]
        return np.sqrt(dx * dx + dy * dy)

def map_2d_to_1d(row, col, cols):
    return row * cols + col

