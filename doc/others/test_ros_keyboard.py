#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Return None all the time. Bad. Maybe needs to connect to Baxter first
'''

import open3d
import numpy as np

import rospy
import baxter_external_devices


# -- Main
if __name__ == "__main__":
    rospy.init_node('node')
    while not rospy.is_shutdown():
        c = baxter_external_devices.getch()
        print c
        rospy.sleep(0.5)