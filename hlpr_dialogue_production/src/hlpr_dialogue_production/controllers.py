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
"""Defines functions to create behavior controllers for dialogue production.

This module contains functions that create controllers for each of the
behaviors that one might want to synchronize with robot speech.  Currently,
this includes looking at a point in space, executing a gesture with pre-
defined waypoints, and a test controller that just plays a beeping sound (and
can be used to check synchronization).
"""


import rospy
import smach
import smach_ros
import actionlib
import random
import os

from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion
from std_msgs.msg import Header
import hlpr_dialogue_production.msg as dialogue_msgs
import hlpr_record_demonstration.msg as record_msgs
import hlpr_lookat.msg as lookat_msgs
import hlpr_lookat.msg


from hlpr_dialogue_production.dialogue import ControllerState
import actionlib_tutorials.msg


def lookat_controller_cb(behavior_name, string_args):
    """Callback to create a LookatAction goal from string arguments
    
    Given a string of arguments (pulled from the behavior tags in a speech
    string), determines whether the robot should look at the base of the
    frame or a point relative to the frame.  Prints a warning message to the
    screen and returns None if the number of arguments is incorrect or they
    are of the wrong type, but does not check that the first argument is a
    valid frame (if it is not, the LookatWaypoints action server will return
    the error instead). 

    Parameters
    ----------
    behavior_name : str
        The name of the behavior being handled (right now, this will always
        be "lookat")
    string_args : list of str
        The arguments that were parsed from the speech string, as strings

    Returns
    -------
    LookatWaypointsGoal
        Goal containing the one point to look at.

    """
    if len(string_args)==1: #have just a frame name
        vec3=Vector3(0.0,0.0,0.0)
    elif len(string_args)==4:
        try:
            point = map(lambda s: float(s), string_args[1:])
        except ValueError:
            rospy.logerr("Invalid arguments to lookat: {}".format(string_args))
            return None
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
    """ Sets up the lookat controller state

    Sets up the lookat controller state to connect to the lookat_waypoints
    action server in hlpr_lookat.  Does not adjust the timing of lookat
    behaviors.

    """
    behaviors=["lookat"]
    time_adj = None
    return ControllerState("LOOKAT_CONTROLLER",behaviors, 
                           "/lookat_waypoints_action_server",
                           hlpr_lookat.msg.LookatWaypointsAction,
                           lookat_controller_cb,time_adj)



def keyframe_playback_controller_cb(behavior_name, string_args):
    """Callback to create a PlaybackKeyframeDemoGoal from string arguments
    
    Given the name of a trajectory, looks up the file associated with the
    name and packs it into a goal for the hlpr_kinesthetic_teaching playback
    server.

    Parameters
    ----------
    behavior_name : str
        The name of the behavior being handled
    string_args : list of str
        The arguments that were parsed from the speech string, as strings
        (not used)

    Returns
    -------
    PlaybackKeyframeDemoGoal
        Goal containing the location of the pickle file to play back

    """
    rospy.loginfo("Got keyframe controller callback")

    pickle_locations={"turn_cup":"/home/eshort/robot_movements/rot_90_deg_joint.pkl",
                      "wave":"/home/eshort/robot_movements/rot_90_deg_joint.pkl"}
    filename = ""

    home_path = os.path.expanduser("~") + "/robot_movements/" + string_args[0] + ".pkl"

    if string_args[0] in pickle_locations:
        filename = pickle_locations[string_args[0]]
    elif os.path.exists(string_args[0]):
        filename = string_args[0]
    elif os.path.exists(home_path):
        filename = home_path
    else:
        pass

    if filename == "":
        rospy.logerr("could not find file for the behavior specified as '" + string_args[0] + "'")
        rospy.logerr("I looked in the following locations:")
        looked_in = pickle_locations.values() + [string_args[0], home_path]
        rospy.logerr(str(looked_in))
    else:
        rospy.loginfo("loading behavior from file " + filename)
    return record_msgs.PlaybackKeyframeDemoGoal(bag_file_name=filename)

def get_keyframe_playback_controller():
    """ Sets up the keyframe playback controller state

    Sets up the keyframe playback controller to connect to the
    playback_demonstration_action_server action server in hlpr_record_demonstration.

    """
    behaviors=["keyframe"]
    time_adj = None
    return ControllerState("KEYFRAME_PLAYBACK_CONTROLLER",behaviors, 
                           "/playback_keyframe_demo",
                           record_msgs.PlaybackKeyframeDemoAction,
                           keyframe_playback_controller_cb,time_adj)

def gesture_controller_cb(behavior_name, string_args):
    """Callback to create a GestureGoal from string arguments
    
    Given a string of arguments (pulled from the behavior tags in a speech
    string), selects the series of named keypoints that define the geature.
    These keypoints are currently defined in the Gesture Action Server (in
    ``gesture_action_server.py``. To add more keypoints, see the documentation
    for that file.

    Parameters
    ----------
    behavior_name : str
        The name of the behavior being handled
    string_args : list of str
        The arguments that were parsed from the speech string, as strings

    Returns
    -------
    GestureGoal
        Gesture goal containing the list of keypoints for the gesture

    """
    
    gesture_poses = {"wave":["wave1","wave2","wave1","wave2"],
                     "shrug":["right_home","shrug","right_home"],
                     "thinking":["right_home","hmm","right_home"]}
    return dialogue_msgs.GestureGoal(poses=gesture_poses[behavior_name])

def get_gesture_controller():
    """ Sets up the gesture controller state

    Sets up the gesture controller state to connect to the gesture action
    server in this package.  Adjusts the timing of the gesture behaviors
    by 1s; you may want to change this for your application.

    """
    behaviors=["wave", "shrug", "thinking"]
    time_adj = {"wave":1.0,
                "shrug":1.0,
                "thinking":1.0}
    
    return ControllerState("GESTURE_CONTROLLER", behaviors, "/HLPR_Gesture",
                           dialogue_msgs.GestureAction,
                           gesture_controller_cb, time_adj)

def test_controller_cb(behavior_name, string_args):
    """Callback to create a FibonacciGoal from string arguments
    
    The test action server just uses the Fibonacci goal from the
    actionlib_tutorials package.  The contents of the goal are not used.

    Parameters
    ----------
    behavior_name : str
        The name of the behavior being handled (right now, this will always
        be "lookat")
    string_args : list of str
        The arguments that were parsed from the speech string, as strings

    Returns
    -------
    FibonacciGoal
        Fibonacci goal with order=0; contents not used.

    """
    return actionlib_tutorials.msg.FibonacciGoal(order=0)

def get_test_controller():
    """ Sets up the test controller state

    Sets up the test controller state to connect to the test action
    server in this package.  Does not adjust the timing of behaviors.

    """
    behaviors=["test"]
    time_adj = {"test":0.0}
    return ControllerState("TEST_CONTROLLER",behaviors,"/test_beep_controller",
                           actionlib_tutorials.msg.FibonacciAction,
                           test_controller_cb, time_adj)
