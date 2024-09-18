#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import math
import numpy as np

def calculate_min_range(angle):
    # Angle limit
    front_angle_limit = math.pi + math.pi / 6
    rear_angle_limit = (math.pi * 2) - math.pi / 2

    # print('front_angle_limit',front_angle_limit)
    # print('rear_angle_limit',rear_angle_limit)

    # Minimum ranges
    front_min_range = 0.20      # Minimum front range 
    rear_min_range = 0.03        # Minimum rear range

    if front_angle_limit < angle < rear_angle_limit:
        return rear_min_range
    else:
        return front_min_range

def callback(scan):
    
    filtered_scan = LaserScan()
    filtered_scan.header = scan.header
    filtered_scan.angle_min = scan.angle_min
    filtered_scan.angle_max = scan.angle_max
    filtered_scan.angle_increment = scan.angle_increment
    filtered_scan.time_increment = scan.time_increment
    filtered_scan.scan_time = scan.scan_time
    filtered_scan.range_min = scan.range_min
    filtered_scan.range_max = scan.range_max

    # Set a threshold for the minimum acceptable range
    # min_range_threshold = 0.25  # meters

    # Filter out ranges that are too close to the sensor
    # filtered_scan.ranges = [r if r >= min_range_threshold else float('inf') for r in scan.ranges]
    
    # Filter out ranges that are too close to the sensor
    filtered_scan.ranges = []

    outliser_filtered = filtered_scan
    for i, r in enumerate(scan.ranges):
        angle = scan.angle_min + i * scan.angle_increment
        min_range_threshold = calculate_min_range(angle)
        if r >= min_range_threshold:
            filtered_scan.ranges.append(r)
            #ranges.append(r)
        else:
            filtered_scan.ranges.append(float('inf'))
            #ranges.append(float('inf'))
        
        # print('angle',angle)
        # print('min range',min_range_threshold)
    
    # Filter out non-numeric values (e.g., NaNs)
    ranges = np.array ( filtered_scan.ranges )
    ranges = ranges[np.isfinite(filtered_scan.ranges)]
    # Statistical filter: remove points outside 1 standard deviation
    mean = np.mean(ranges)
    std_dev = np.std(ranges)

    std_dev  = std_dev*1.6
    filtered_ranges = [r if (mean - std_dev <= r <= mean + std_dev) else float('inf') for r in filtered_scan.ranges]
    outliser_filtered.ranges = filtered_ranges
    
    outliser_filtered.intensities = scan.intensities

    pub.publish(outliser_filtered)
    
if __name__ == '__main__':
    rospy.init_node('laser_filter_node', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, callback)
    pub = rospy.Publisher('/scan/filtered', LaserScan, queue_size=10)
    rospy.spin()
