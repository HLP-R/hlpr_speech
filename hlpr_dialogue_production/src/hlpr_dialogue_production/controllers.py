#!/usr/bin/env python

# Copyright (c) 2017, Elaine Short, SIM Lab
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# 
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# 
# * Neither the name of the SIM Lab nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rospy
import smach
import smach_ros
import actionlib
import random

from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion
from std_msgs.msg import Header
import hlpr_dialogue_production.msg as dialogue_msgs


from hlpr_dialogue_production.dialogue import ControllerState
import hlpr_lookat.msg


import actionlib_tutorials.msg


def lookat_controller_cb(behavior_name, string_args):
    if len(string_args)==1: #have just a frame name
        vec3=Vector3(0.0,0.0,0.0)
    elif len(string_args)==4:
        point = map(lambda s: float(s), string_args[1:])
        vec3=Vector3(point[0],point[1],point[2])
    else:
        rospy.logerr("Invalid arguments to lookat: {}".format(string_args))
        return None

    frame = string_args[0]
 
    pos = TransformStamped()
    pos.child_frame_id = frame
    pos.transform = Transform()
    pos.transform.translation = vec3
    pos.header = Header()
    waypoints = [pos]
    waytimes = [rospy.Duration(0.0)]
    return hlpr_lookat.msg.LookatWaypointsGoal(waypoints,waytimes)

def get_lookat_controller():
    behaviors=["lookat"]
    time_adj = None
    return ControllerState("LOOKAT_CONTROLLER",behaviors, 
                           "/lookat_waypoints_action_server",
                           hlpr_lookat.msg.LookatWaypointsAction,
                           lookat_controller_cb,time_adj)

def gesture_controller_cb(behavior_name, string_args):
    gesture_poses = {"wave":["wave1","wave2","wave1","wave2"],
                     "shrug":["right_home","shrug","right_home"],
                     "thinking":["right_home","hmm","right_home"]}
    return dialogue_msgs.GestureGoal(poses=gesture_poses[behavior_name])

def get_gesture_controller():
    behaviors=["wave", "shrug", "thinking"]
    time_adj = {"wave":1.0,
                "shrug":1.0,
                "thinking":1.0}
    
    return ControllerState("GESTURE_CONTROLLER", behaviors, "/HLPR_Gesture",
                           dialogue_msgs.GestureAction,
                           gesture_controller_cb, time_adj)

def test_controller_cb(behavior_name, string_args):
    return actionlib_tutorials.msg.FibonacciGoal(order=20)

def get_test_controller():
    behaviors=["test"]
    time_adj = {"test":0.0}
    return ControllerState("TEST_CONTROLLER",behaviors,"/test_beep_controller",
                           actionlib_tutorials.msg.FibonacciAction,
                           test_controller_cb, time_adj)
