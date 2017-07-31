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
import yaml
import smach_ros
import actionlib
import argparse
from hlpr_dialogue_production.dialogue import SmachWrapper, ControllerState
import hlpr_dialogue_production.controllers as controller_gen
import hlpr_dialogue_production.msg as dialogue_msgs

class HLPRDialogueAction():
    def __init__(self, use_tts, phrase_file=None, voice="Kimberly", debug=False):
        self._as = actionlib.SimpleActionServer("HLPR_Dialogue", dialogue_msgs.DialogueActAction, execute_cb=self.execute_cb, auto_start=False)
        self._use_tts = use_tts

        if phrase_file!=None:
            with open(phrase_file, 'r') as f:
                s = f.read()
                self._phrases = yaml.load(s)
        else:
            self._phrases=None

        self._s = SmachWrapper(self._use_tts,self._phrases,self._setup_controllers(),voice,debug)

        if debug:
            self._condition = "debug"
        elif use_tts and phrase_file!=None:
            self._condition = "tts_fallback"
        elif use_tts:
            self._condition = "tts_only"
        elif phrase_file!=None:
            self._condition = "file_only"
        else:
            self._condition = "online_behavior"

        self._sis = smach_ros.IntrospectionServer('hlpr_dialogue_smach_server', self._s.get_sm(), '/SPEECH_SM')
        self._sis.start()
        self._as.start()
        rospy.loginfo("Dialogue server ready to play speech")
        
    def _setup_controllers(self):
        controllers=[]
        controllers=[controller_gen.get_test_controller(),
                     controller_gen.get_lookat_controller(),
                     controller_gen.get_gesture_controller(),
                    ]
        return controllers

    def execute_cb(self,goal):
        userdata = {}
        if self._condition=="debug":
            userdata["key_or_marked_text"]=goal.text_or_key
            userdata["behaviors"]=yaml.load(goal.behavior_yaml)
            userdata["wav_file_loc"]=goal.audio_file
        elif self._condition=="tts_fallback":
            userdata["key_or_marked_text"]=goal.text_or_key
        elif self._condition=="tts_only":
            userdata["marked_text"]=goal.text_or_key
        elif self._condition=="file_only":
            userdata["key"]=goal.text_or_key
        else:
            userdata["behaviors"]=yaml.load(goal.behavior_yaml)
            userdata["wav_file_loc"]=goal.audio_file

        self._s.standalone_start(userdata)
        success=True
        while self._s.is_running():
            if self._as.is_preempt_requested():
                self._s.preempt()
                success = False
                break
            self._as.publish_feedback(dialogue_msgs.DialogueActFeedback(self._s.get_active_states()))
            rospy.sleep(0.01)
        
        if success:
            self._as.set_succeeded(dialogue_msgs.DialogueActResult(self._s.get_outcome()))
        else:
            self._as.set_preempted()
        
    def spin(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.01)
        self._sis.stop()

if __name__=="__main__":
    rospy.init_node("smach_dialogue_server")
    
    parser=argparse.ArgumentParser(description="An action server to run dialogue behaviors for the HLPR robot")
    parser.add_argument('-v', '--voice', help="Which voice to use with TTS. Child Voices: Ivy, Justin; Adult Voices: Salli, Joey, Kimberly, Kendra, Eric, Jennifer; Silly Voices: Chipmunk", default="Kimberly")
    parser.add_argument('-t', '--use-tts', help="Enable text-to-speech (online)", action='store_true')
    parser.add_argument('-p', '--phrase-file', help="Phrase file for pre-generated speech", nargs="?", default=None)
    parser.add_argument('-d', '--debug', help="Use debug mode. THIS WILL OVERRIDE -v, -t, AND -p FLAGS!", action='store_true')

    args = parser.parse_known_args()[0]


    server = HLPRDialogueAction(args.use_tts,args.phrase_file,args.voice, args.debug)
    server.spin()
