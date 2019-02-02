#!/usr/bin/env python3
# _*_ coding: utf-8 _*_
""" bead_projection.py
Produce projection images of the bead of different poses.

Yongming Qin
2018/01/27
"""
from gazebo_msgs.srv import ModelState
import rospy
import numpy as numpy

from geometry_msgs.msg import Quaternion

rospy.wait_for_service('/gazebo/set_model_state')
func = rospy.ServiceProxy('/gazebo/set_model_state', ModelState)

srv = ModelState()
srv.request.model_name = "bead"
srv.request.pose =
srv.request.orientation
srv.reqest.twist
srv.request.angular
srv.request.
resp = func(srv)