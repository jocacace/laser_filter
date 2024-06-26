#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

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
    min_range_threshold = 0.25  # meters

    # Filter out ranges that are too close to the sensor
    filtered_scan.ranges = [r if r >= min_range_threshold else float('inf') for r in scan.ranges]
    filtered_scan.intensities = scan.intensities

    pub.publish(filtered_scan)
    
if __name__ == '__main__':
    rospy.init_node('laser_filter_node', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, callback)
    pub = rospy.Publisher('/scan/filtered', LaserScan, queue_size=10)
    rospy.spin()
