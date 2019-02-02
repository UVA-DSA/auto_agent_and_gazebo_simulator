#!/usr/bin/env python3
# _*_ coding: utf-8 _*_
""" eef_projection.py
Produce projection images of the eef of different poses and joint values.

Yongming Qin
2018/01/27
"""
from gazebo_msgs.srv import SetModelConfiguration
import rospy
import numpy as numpy

rospy.wait_for_service('set_model_configuration')
func = rospy.ServiceProxy('set_model_configuration', SetModelConfiguration)

srv = SetModelConfiguration()
srv.request.model_name = "bead"
srv.request.urdf_param
srv.request.joint_names.append()
srv.reqest.joint_positions.
resp = func()