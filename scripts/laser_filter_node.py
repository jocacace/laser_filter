#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import math
import numpy as np


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
    filtered_scan.ranges = []

    # Document and check the parametrs
    threshold_factor = 0.8
    max_range = 2.0

    outliser_filtered = filtered_scan
    for i, r in enumerate(scan.ranges): 

        angle = scan.angle_min + i * scan.angle_increment
        
        if ( angle < math.pi / 2.0 or angle > 4.7 ):
            if r > 0.25: 
                filtered_scan.ranges.append(r)
            else:
                filtered_scan.ranges.append(float('inf'))
        else: 
            if r > 0.1:
                filtered_scan.ranges.append(r)
            else:
                filtered_scan.ranges.append(float('inf'))

    #outliser_filtered.ranges = filtered_ranges
    #outliser_filtered.intensities = scan.intensities
    #pub.publish(filtered_scan)

    
    ranges = np.array ( filtered_scan.ranges )
    #ranges = ranges[np.isfinite(filtered_scan.ranges)]

    x_sum = 0.0
    y_sum = 0.0
    valid_points = 0

    # Calculate the centroid of the points
    for i, r in enumerate(ranges):
        if r > 0 and r < float('inf'):  # Considera solo i punti validi
            angle = scan.angle_min + i * scan.angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            x_sum += x
            y_sum += y
            valid_points += 1

    if valid_points == 0:
        rospy.logwarn(f"No valid points in the ranges.")
        return 

    x_centroid = x_sum / valid_points
    y_centroid = y_sum / valid_points

    if x_centroid is None or y_centroid is None:
        rospy.logwarn("Nessun punto valido per calcolare il centroide.")
        return


    max_radius = 4.0

    # Create a new range considering the distance from the centroid
    distances_to_centroid = []
    valid_ranges = []
    for i, r in enumerate(filtered_scan.ranges):
        if r > 0 and r < float('inf'):  # Punti validi
            angle = scan.angle_min + i * scan.angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)

            # Calcola la distanza tra il punto e il centroide
            distance_to_centroid = math.sqrt((x - x_centroid)**2 + (y - y_centroid)**2)
            distances_to_centroid.append(distance_to_centroid)
            valid_ranges.append(r)
        else:
            valid_ranges.append(0.0)
            distances_to_centroid.append(0.0)
        
    
    filtered_ranges = []
    for i, r in enumerate(valid_ranges):
    
        if distances_to_centroid[i] <= max_radius:
            filtered_ranges.append(valid_ranges[i])
        else:
            filtered_ranges.append(0.0)
    
    outliser_filtered.ranges = filtered_ranges
    outliser_filtered.intensities = scan.intensities
    pub.publish(outliser_filtered)
    
    
    
if __name__ == '__main__':
    rospy.init_node('laser_filter_node', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, callback)
    pub = rospy.Publisher('/scan/filtered/bags', LaserScan, queue_size=10)
    rospy.spin()
