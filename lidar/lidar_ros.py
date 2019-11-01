# TODO: ROS Integration
# Check out http://wiki.ros.org/xv_11_laser_driver/Tutorials/Running%20the%20XV-11%20Node
# Add ROS publisher to XV-11 test
# ----------------------------------
# TODO: Simple rock avoidance
# Convert data into (x,y) points
# Select data that is in field of view (~90 deg in front of hexapod)
# Filter out zeros
# Draw best fit line
# Points significantly closer than best fit line represent obstacles
# Select gaps correctly spaced for hexapod to walk through
# Rotate hexapod to align with gaps
# ----------------------------------
# TODO: 3D terrain navigation
# Compute height from distance and pitch (lidar + accel)
# Filter out datapoints above horizon (too far)
# Combine multiple sweeps and find best fit line for slope at each angle
# Find slope shallow enough for hexapod to walk
# Adjust hexapod gait to compensate for slope

import numpy as np

class LidarNode():

    def __init__(self):
        self.vals = np.zeros(360)
        self.points = np.zeros((360,2))

    def publish(self, angle, dist_mm):
        self.vals[angle] = dist_mm
        self.points[angle,0] = np.cos(np.radians(angle))*self.vals[angle]
        self.points[angle,1] = np.sin(np.radians(angle))*self.vals[angle]
        print(angle,dist_mm)

if __name__ == '__main__':
    node = LidarNode()
    node.publish(90, 1)