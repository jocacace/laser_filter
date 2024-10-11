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

def filter_outliers(distances, threshold_factor=1.5):
    """
    Filtra gli outlier vicini usando la distanza media e uno scarto basato su una soglia.
    """
    mean_distance = np.mean(distances)
    std_distance = np.std(distances)
    
    lower_bound = mean_distance - threshold_factor * std_distance

    # Mantieni solo i punti che sono sopra il limite inferiore (rimuovi quelli troppo vicini)
    return [d for d in distances if d >= lower_bound]


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
        min_range_threshold = calculate_min_range(angle)

        if r >= min_range_threshold:
            filtered_scan.ranges.append(r)
        else:
            filtered_scan.ranges.append(float('inf'))
        

    ranges = np.array ( filtered_scan.ranges )
    ranges = ranges[np.isfinite(filtered_scan.ranges)]

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


    max_radius = 2.0

    # Create a new range considering the distance from the centroid
    distances_to_centroid = []
    valid_ranges = []
    for i, r in enumerate(scan.ranges):
        if r > 0 and r < float('inf'):  # Punti validi
            angle = scan.angle_min + i * scan.angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)

            # Calcola la distanza tra il punto e il centroide
            distance_to_centroid = math.sqrt((x - x_centroid)**2 + (y - y_centroid)**2)
            distances_to_centroid.append(distance_to_centroid)
            valid_ranges.append(r)

    # Filtra gli outlier vicini
    filtered_distances = filter_outliers(distances_to_centroid, threshold_factor)
    filtered_ranges = []
    for i, r in enumerate(valid_ranges):
        if distances_to_centroid[i] in filtered_distances and distances_to_centroid[i] <= max_radius:
            filtered_ranges.append(r)
        else:
            filtered_ranges.append(0.0)

    outliser_filtered.ranges = filtered_ranges
    outliser_filtered.intensities = scan.intensities
    pub.publish(outliser_filtered)
    
if __name__ == '__main__':
    rospy.init_node('laser_filter_node', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, callback)
    pub = rospy.Publisher('/scan/filtered', LaserScan, queue_size=10)
    rospy.spin()
