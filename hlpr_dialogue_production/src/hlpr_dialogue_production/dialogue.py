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
import wave
import yaml
import threading
import actionlib
import contextlib
from sound_play.libsoundplay import SoundClient
from cordial_tts.cordial_tts import CoRDialTTS

class Synchronizer():
    def __init__(self):
        self.started = False
        self.time = None

    def start(self):
        self.started = True
        self.time = rospy.Time.now()
    
    def reset(self):
        self.started = False
        self.time = None

class ControllerState(smach.State):
    def __init__(self, name, behaviors, topic, action_type, arg_list_to_goal_cb, behavior_time_adj=None):
        smach.State.__init__(self,outcomes=["preempted","done"],
                             input_keys=["ordered_behaviors"])
        self.name = name
        self._can_handle = behaviors
        self._cb = arg_list_to_goal_cb
        self._client = actionlib.SimpleActionClient(topic,action_type)
        self._sync = None
        if behavior_time_adj!=None:
            self._time_adj = behavior_time_adj
        else:
            self._time_adj = {}

    def setup_sync(self,synchronizer):
        self._sync=synchronizer

    def execute(self,userdata):
        ordered_behaviors = filter(lambda b: b["id"] in self._can_handle,
                                          userdata.ordered_behaviors)

        while not (self._sync!=None and self._sync.started) and not self.preempt_requested():
            rospy.sleep(0.001)

        if self.preempt_requested():
            self._client.cancel_all_goals()
            self.service_preempt()
            return "preempted"

        goal_sent=False
        for b in ordered_behaviors:
            if b["id"] in self._time_adj:
                start = b["start"]-self._time_adj[b["id"]]
            else:
                start = b["start"]

            goal = self._cb(b["id"],b["args"])
            if goal==None:
                continue
            while rospy.Time.now()-self._sync.time < rospy.Duration.from_sec(start) and not self.preempt_requested():
                rospy.sleep(0.001)

            if self.preempt_requested():
                self._client.cancel_all_goals()
                self.service_preempt()
                return "preempted"

            self._client.send_goal(goal)
            goal_sent = True
        if goal_sent:
            while not self._client.wait_for_result(rospy.Duration(0.05)):
                if self.preempt_requested():
                    self._client.cancel_all_goals()
                    self.service_preempt()
                    return "preempted"
        return "done"

class TTSSpeechStart(smach.State):
    def __init__(self,voice="Kimberly"):
        smach.State.__init__(self,outcomes=["done"],
                             input_keys=["marked_text"],
                             output_keys=["text","ordered_behaviors","wav_file"])
        self._tts = CoRDialTTS(voice)

    def execute(self,userdata):
        text, behaviors = self._tts.extract_behaviors(userdata.marked_text)
        wav_file = None
        userdata.ordered_behaviors = sorted(behaviors,key = lambda b: b["start"])
        userdata.text = text
        userdata.wav_file = wav_file
        return "done"

class TTSFallbackSpeechStart(smach.State):
    def __init__(self, phrases, voice="Kimberly"):
        smach.State.__init__(self,outcomes=["done"],
                             input_keys=["key_or_marked_text"],
                             output_keys=["text","ordered_behaviors","wav_file"])
        self._tts = CoRDialTTS(voice)
        self._phrases = phrases

    def execute(self,userdata):
        if userdata.key_or_marked_text in self._phrases:
            phrase = self._phrases[userdata.key_or_marked_text]
            if "text" in phrase:
                text = phrase["text"]
            else:
                text = None
            wav_file = phrase["file"]
            behaviors = phrase["behaviors"]
        else:
            text, behaviors = self._tts.extract_behaviors(userdata.key_or_marked_text)
            wav_file = None

        userdata.ordered_behaviors = sorted(behaviors,key = lambda b: b["start"])
        userdata.text = text
        userdata.wav_file = wav_file
        return "done"

class FileSpeechStart(smach.State):
    def __init__(self, phrases):
        smach.State.__init__(self,outcomes=["done","not_found"],
                             input_keys=["key"],
                             output_keys=["text","ordered_behaviors","wav_file"])
        self._phrases = phrases

    def execute(self,userdata):
        if userdata.key not in self._phrases:
            return "not_found"
        phrase = self._phrases[userdata.key]
        if "text" in phrase:
            text = phrase["text"]
        else:
            text = None
        wav_file = phrase["file"]
        behaviors = phrase["behaviors"]
        
        userdata.ordered_behaviors = sorted(behaviors,key = lambda b: b["start"])
        userdata.text = text
        userdata.wav_file = wav_file
        return "done"

class NoPrepSpeechStart(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=["done","missing_info"],
                             input_keys=["behaviors","wav_file_loc"],
                             output_keys=["text","ordered_behaviors","wav_file"])

    def execute(self,userdata):
        if userdata.behaviors==None:
            return "missing_info"
        if userdata.wav_file==None:
            return "missing_info"
        behaviors = userdata.behaviors
        text = None
        wav_file = userdata.wav_file

        userdata.ordered_behaviors = sorted(behaviors,key = lambda b: b["start"])
        userdata.text = text
        userdata.wav_file = wav_file
        return "done"

       

class SpeechState(smach.State):
    def __init__(self, use_tts, synchronizer, phrases=None, voice="Kimberly"):
        smach.State.__init__(self,outcomes=["preempted","no_audio","done"],
                             input_keys=["text","wav_file"])
        self._tts = use_tts
        if use_tts:
            self._talker = CoRDialTTS(voice)
        self._sound_client = SoundClient()
        self._sync = synchronizer
 
    def execute(self,userdata):
        rospy.loginfo("Saying: {}".format(userdata.text))
        if userdata.wav_file==None:
            if userdata.text!=None and len(userdata.text)!=0:
                speech_duration = self._talker.say(userdata.text,wait=False)
                self._sync.start()
            else:
                return "no_audio"
        else:
            wav_file = userdata.wav_file
            with contextlib.closing(wave.open(wav_file,'r')) as f:
                frames=f.getnframes()
                rate=f.getframerate()
                speech_duration=frames/float(rate)
            self._sound_client.playWave(wav_file)
            self._sync.start()
        
        while rospy.Time.now()-self._sync.time<rospy.Duration(speech_duration):
            if self.preempt_requested():
                if self._tts:
                    self._talker.shutup()
                else:
                    self._sound_client.stopAll()
                self.service_preempt()
                return "preempted"
        return "done"

class SmachWrapper():
    def __init__(self, use_tts, phrases=None, controllers=None, voice="Kimberly"):
        if use_tts and phrases!=None:
            condition = "tts_fallback"
            outcomes = ["preempted","done"]
            #input_keys = ["key_or_marked_text"]
        elif use_tts:
            condition = "tts_only"
            outcomes = ["preempted","done"]
            #input_keys = ["marked_text"]
        elif phrases!=None:
            condition = "file_only"
            outcomes = ["preempted","not_found","done"]
            #input_keys = ["key"]
        else:
            condition = "fully_online"
            outcomes = ["preempted","missing_info", "done"]
            #input_keys = ["behaviors","wav_file_loc"]

        rospy.loginfo("Starting SMACH Wrapper with outcomes {}".format(outcomes))

        self._sm = smach.StateMachine(outcomes=outcomes)#,
                                      #input_keys=input_keys)

        self._sync = Synchronizer()


        def cc_child_term_cb(outcome_map):
            others = []
            for name, outcome in outcome_map.items():
                others.append(outcome)
            if all(others):
                return True

        def cc_outcome_cb(outcome_map):
            if outcome_map["SPEECH"]=="done":
                self._outcome="done"
                return "done"
            else:
                self._outcome="preempted"
                return "preempted"

        self._cc = smach.Concurrence(outcomes=["preempted","done"],
                               default_outcome = "done",
                               input_keys = ["text","wav_file","ordered_behaviors"],
                               child_termination_cb=cc_child_term_cb,
                               outcome_cb=cc_outcome_cb)
        # ControllerState params: name, behaviors, topic, action_type, arg_list_to_goal_cb, synchronizer, behavior_time_adj=None
        

        with self._cc:
            smach.Concurrence.add("SPEECH", SpeechState(use_tts, self._sync, phrases,voice),
                            remapping={"text":"text",
                                       "wav_file":"wav_file"})
            if controllers != None:
                for c in controllers:
                    c.setup_sync(self._sync)
                    smach.Concurrence.add(c.name, c,
                                    remapping={"ordered_behaviors":"ordered_behaviors"})
    
                

        with self._sm:
            if condition=="tts_fallback":
                smach.StateMachine.add("START",TTSFallbackSpeechStart(phrases),
                                       transitions={"done":"DIALOGUE"},
                                       remapping={"key_or_marked_text":"key_or_marked_text",
                                                  "text":"text",
                                                  "ordered_behaviors":"ordered_behaviors",
                                                  "wav_file":"wav_file"})
            elif condition=="tts_only":
                smach.StateMachine.add("START",TTSSpeechStart(),
                                       transitions={"done":"DIALOGUE"},
                                       remapping={"marked_text":"marked_text",
                                                  "text":"text",
                                                  "ordered_behaviors":"ordered_behaviors",
                                                  "wav_file":"wav_file"})
            elif condition=="file_only":
                smach.StateMachine.add("START",FileSpeechStart(phrases),
                                       transitions={"done":"DIALOGUE",
                                                    "not_found":"not_found"},
                                       remapping={"key":"key",
                                                  "text":"text",
                                                  "ordered_behaviors":"ordered_behaviors",
                                                  "wav_file":"wav_file"})
            else:
                smach.StateMachine.add("START",NoPrepSpeechStart(),
                                       transitions={"done":"DIALOGUE",
                                                    "missing_info":"missing_info"},
                                       remapping={"behaviors":"behaviors",
                                                  "wav_file_loc":"wav_file_loc",
                                                  "text":"text",
                                                  "ordered_behaviors":"ordered_behaviors",
                                                  "wav_file":"wav_file"})
                
            smach.StateMachine.add("DIALOGUE",self._cc,
                                   transitions={"done":"done",
                                                "preempted":"preempted"},
                                   remapping={"wav_file":"wav_file",
                                              "ordered_behaviors":"ordered_behaviors",
                                              "text":"text"})
    def get_sm(self):
        return self._sm

    def _service_preempt_for_children(self,sm):
        sm.service_preempt()
        try:
            children = sm.get_children().values()
            for child in children:
                self._service_preempt_for_children(child) 
        except AttributeError:
            return

    def standalone_start(self):
        self.reset()
        self._t = threading.Thread(target=self._sm.execute)
        self._t.start()

    def reset(self):
        self._sync.reset()
        self._service_preempt_for_children(self._sm)
        self._outcome=None

    def get_outcome(self):
        return self._outcome

    def preempt(self):
        #self.preempter.preempt()
        self._sm.request_preempt()
        self._t.join()

    def is_running(self):
        return self._t.isAlive()

    def get_active_states(self):
        if self._sm.is_running():
            return self._sm.get_active_states()
        else:
            return None
