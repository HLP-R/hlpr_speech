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
import moveit_commander
import moveit_msgs.msg
import math
import copy
import actionlib
from hlpr_manipulation_utils.arm_moveit2 import ArmMoveIt
from hlpr_manipulation_utils.manipulator import Arm
import hlpr_dialogue_production.msg as dialogue_msgs

#A gesture is a trajectory through a set of named poses 
#(for now, no parameterized gestures, e.g., deixis
#this lets us save those trajectories for speed.
class RobotArmGestureServer():
    def __init__(self):
        self.arm = Arm()
        self._arm_planner = ArmMoveIt()
        self._joints = self._arm_planner.get_active_joints()
        self._saved_trajs = {}
        self._saved_poses = {}
        self._current_pose = None

        self._as = actionlib.SimpleActionServer("HLPR_Gesture",dialogue_msgs.GestureAction, execute_cb=self.execute_cb, auto_start=False)

    def start(self):
        self._as.start()

    def execute_cb(self, goal):
        success = True
        preempted = False
        for pose in goal.poses:
            rospy.loginfo("Going to pose {}".format(pose))
            if self._as.is_preempt_requested():
                preempted=True
                break
            success = success and self.goto(pose)
            self._as.publish_feedback(dialogue_msgs.GestureFeedback(current_pose=pose))
        
        if not preempted:
            self._as.set_succeeded(dialogue_msgs.GestureResult(success=success))
        else:
            self._as.set_preempted()

    def goto(self, pose_name):
        return self._arm_planner.move_to_joint_pose(self.joints_from_pose(pose_name))

        # stuff below might give us the chance to cache plans...
        ''''if self._current_pose==None:
            success = self.goto_joints(self.joints_from_pose(pose_name))
        else:
            if not (self._current_pose in self._saved_trajs):
                target = self.joints_from_pose(pose_name)
                plan = self._arm_planner.plan_joint_pos(target)
                self._saved_trajs[self._current_pose]={pose_name:plan}
            else:
                if not (pose_name in self._saved_trajs[self._current_pose]):
                    target = self.joints_from_pose(pose_name)
                    plan = self._arm_planner.plan_joint_pos(target)
                    self._saved_trajs[self._current_pose][pose_name]=plan
            success = self._send_plan(self._saved_trajs[self._current_pose][pose_name])
        if success:
            self._current_pose = pose_name'''

    def joints_from_pose(self, pose_name):
        if not pose_name in self._saved_poses:
            rospy.logerr("Unknown pose: {}".format(pose_name))
        return copy.copy(self._saved_poses[pose_name])

    def get_params(self):
        print self._arm_planner.robot.get_current_variable_values()

    def get_current_pose(self):
        pose = dict(zip(self._joints,self._arm_planner.get_current_pose()))
        return pose

    def random_move(self):
        self._arm_group.set_random_target()
        plan2 = self._arm_group.plan()
        self._arm_group.execute(plan2)

    def save_pose(self, name, pose, gen_plans=False):
        self._saved_poses[name]=pose

if __name__=="__main__":
    rospy.init_node("hlpr_gesture_server")
    m = RobotArmGestureServer()

    m.save_pose("right_home", [0.3116921319979644, 4.533942390452264, 1.4808211755804646, 5.653696016981127, -1.9270754907123564, -4.045499061077166])
    #m.save_pose("left_home", [2.7341893926390792, 4.320231018238331, 1.244929656271399, 3.6820704155550787, -4.353947168563705, 3.9856245846616094])

    m.save_pose("close_test", [2.8341893926390792, 4.320231018238331, 1.244929656271399, 3.6820704155550787, -4.353947168563705, 3.9856245846616094])
    
    m.save_pose("hmm", [1.0008403637585435, 4.187781938178258, 1.3060713538776199, -0.7018143600614692, -1.3620428741204904, 2.2839403952261854])

    m.save_pose("shrug", [0.7307717833993168, 4.567836684906634, 1.059644788555091, 0.8238771260624587, -2.5188028399948124, 1.0415708104338988])

    base_joints = m.joints_from_pose("right_home")
    idx = m._joints.index("right_wrist_1_joint")
    base_joints[idx] = base_joints[idx]+0.3
    
    idx = m._joints.index("right_wrist_2_joint")
    base_joints[idx] = base_joints[idx]+0.1

    m.save_pose("wave1",base_joints)
    
    base_joints = m.joints_from_pose("right_home")
    idx = m._joints.index("right_wrist_1_joint")
    base_joints[idx] = base_joints[idx]-0.3
    
    idx = m._joints.index("right_wrist_2_joint")
    base_joints[idx] = base_joints[idx]-0.1

    m.save_pose("wave2",base_joints)
    #print m._arm_planner.get_current_pose()

    m.start()
    rospy.spin()
