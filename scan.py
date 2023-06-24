#! /usr/bin/env python3

import rospy
import numpy as np
import time
import csv
import os
from sensor_msgs.msg import LaserScan, PointCloud
import math
import time


def get_today():
    now = time.localtime()
    s = "%04d-%02d-%02d %02d:%02d" %(now.tm_year,now.tm_mon,now.tm_mday,now.tm_hour,now.tm_min)
    return s
    
def make_folder(folder_name):
    if not os.path.isdir(folder_name):
        os.mkdir(folder_name)
         
def callback(msg):
    now = time.localtime()
    
    if now.tm_sec%10 != 5:
        return 0

    data = list(msg.ranges)
    n = len(data)
    print_num = 10
    d_angle = int(360/print_num)
    d_num = math.floor(n/print_num)

    theta_list = []
    range_list = []

    print(msg.header.seq)        
    for k in range(print_num):
        theta_list.append(k*d_angle)
        range_list.append(data[k*d_num])

    for i in range(print_num):
        print("{:4}' ".format(theta_list[i]))

    for i in range(print_num):
        print("{:4}m ".format(round(range_list[i], 1)))

    with open(filename,'a') as file:
        write = csv.writer(file)
        write.writerow(data)

root_dir = "/home/pi/scandata"
today = get_today()

make_folder(root_dir)
filename = (root_dir + "/" + 'data_{}.csv'.format(today))

rospy.init_node('scan_values')
sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()
