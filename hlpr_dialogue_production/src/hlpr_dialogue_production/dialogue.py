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
    """ Object that can be passed to all controllers to synchronize their start

    A single Synchronizer object is passed to all controllers so that the
    speech state can signal when the audio has begun to play.  This is needed
    because online text-to-speech has to download the audio from the internet
    before it can begin to play, leading to a variable start time for the
    speech audio.
    """
    
    def __init__(self):
        """ Creates a Synchronizer object"""
        self.started = False
        self.time = None

    def start(self):
        """ Sets the state of the Synchronizer to started and saves the time"""
        self.started = True
        self.time = rospy.Time.now()
    
    def reset(self):
        """ Resets the state of the Synchronizer to not started"""
        self.started = False
        self.time = None

class ControllerState(smach.State):
    """ Interface to action servers controlling robot behavior

    A ControllerState is a smach.State that sends requests to a specific
    action server at the appropriate times.

    .. warning:: After creating the object, you *must* call ``setup_sync`` to    provide the state with a Synchronizer. The Synchronizer is not included in    the constructor so that the object can be created in ``controllers.py`` or    elsewhere in code and passed to the SmachWrapper state, which will then set    up the Synchronizers and assemble the dialogue act state machine.

    
    **Input keys**
        * ordered_behaviors : list of dict
            An ordered list of all the behaviors the controller should play. A
            behavior is a dict with keys "start", "id", and "args", indicating
            the start time (after speech begins), name of the behavior, and any
            arguments to the behavior, respectively.

    **Outcomes**
        * preempted
            The behavior was preempted before playing all the behaviors in the list
            to completion.
        * done
            The controller played all of the behaviors in the list.

    """
    def __init__(self, name, behaviors, topic, action_type, arg_list_to_goal_cb, behavior_time_adj=None):
        """ Constructor

        Creates the ControllerState with the given name, able to handle the
        given behaviors by sending the arguments to the arg_list_to_goal_cb
        with a time adjustment factor of behavior_time_adj (assigned on a per-
        behavior basis).  By convention, the name should be in all caps, and
        must not conflict with any other names in the state machine containing
        this state.  If using the SmachWrapper, the names "SPEECH", "START",
        "DIALOGUE" are taken.


        Parameters
        ----------
        name : str
            The name of this state in the state machine
        
        behaviors : list of str
            The behaviors (from the speech string tags) that this controller
            can handle
        
        topic : str
            The topic for the action server that this controller calls
        
        action_type : ROS Action Type
            The type for the action server that this controller calls
        
        arg_list_to_goal_cb : function
            A function taking the name of the behavior and a list of arguments
            and returning a goal for the action of type ``action_type``
        
        behavior_time_adj : dict
            A mapping from behavior names (from ``behaviors``) to times (in s).
            The controller will send the action goal that much earlier.
            Negative values will cause the controller to call the action
            server later.
        
        """
        
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
        """ Provides the controller with a Synchronizer object

        This provides the controller with information about when the speech
        audio has begun.  This will only work if the controller and speech
        state share the same Synchronizer object.

        Parameters
        ----------
        synchronizer : Synchronizer
           A synchronizer object.  All controllers, including the speech state,
           should share the same synchronizer.
        
        """
        self._sync=synchronizer

    def execute(self,userdata):
        if userdata.ordered_behaviors==None:
            rospy.logwarn("Controller {} got empty behavior list".format(self.name))
            return "done"

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
    """ Speech prep state when using TTS

    State to process marked text to ordered behaviors, unmarked text, and an
    empty wav_file, that can be passed on to the Speech state.


    **Input keys**
        * marked_text : str
            Text marked up with <behavior tags>.  Behaviors will be synced to
            the word following the tag.

    **Output keys**
        * text : str
            Text marked up with <behavior tags>.  Behaviors will be synced to
            the word following the tag.
        * ordered_behaviors : list of dict
            A list of behavior dictionaries.  A behavior has the keys "id",
            "start", and "args". Ordered by start time.
        * wav_file : str
            Always None

    **Outcomes**
        * done
            Finished fetching behaviors

    """
    def __init__(self,voice="Kimberly"):
        """ Constructor

        Initializes TTS for speech using Amazon Polly with the given voice

        Parameters
        -----------
        voice : str, optional
            Which Amazon Polly voice to use. Defaults to Kimberly
        """
        
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
    """ Speech prep from file with online TTS fallback

    State to read text, wave file, and ordered behaviors from a phrase file,
    given a key into that file. If the key isn't found, uses online TTS using
    Amazon Polly.  This is useful for debugging or if there are a small number
    of phrases that can't be known in advance.

    **Input keys**
        * key_or_marked_text : str
            Either a key into the phrase file or text marked up with
            <behavior tags>.  The state tries to read from the phrase file
            with this string.  If it's not found, assume it is marked up text
            and generate the audio with TTS

    **Output keys**
        * text : str
            Text marked up with <behavior tags>.  Behaviors will be synced to
            the word following the tag.
        * ordered_behaviors : list of dict
            A list of behavior dictionaries.  A behavior has the keys "id",
            "start", and "args". Ordered by start time.
        * wav_file : str
            None if the audio needs to be fetched, otherwise the path to the
            audio file.

    **Outcomes**
        * done
            Finished fetching behaviors
    
    """
    def __init__(self, phrases, voice="Kimberly"):
        """ Constructor

        Initializes TTS for speech using Amazon Polly with the given voice, and
        reads in the phrase file.

        Parameters
        -----------
        voice : str, optional
            Which Amazon Polly voice to use. Defaults to Kimberly

        phrases : str
            Path to the phrase file to use.
        """
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

class DebugSpeechStart(smach.State):
    """ Debugging speech state with no audio

    Prints the values of the provided userdata, allowing for debugging.  Doesn't
    actually play any sound.

    **Input keys**
        * behaviors : list of dict
             A list of behavior dictionaries.  A behavior has the keys "id",
            "start", and "args".
        * wav_file_loc : str
             The path to the audio file to play
        * key_or_marked_text : str
            Either a key into the phrase file or text marked up with
            <behavior tags>.  

    **Output keys**
        * text : str
            Text marked up with <behavior tags>.  Behaviors will be synced to
            the word following the tag.
        * ordered_behaviors : list of dict
            A list of behavior dictionaries.  A behavior has the keys "id",
            "start", and "args". Ordered by start time.
        * wav_file : str
            None if the audio needs to be fetched, otherwise the path to the
            audio file.

    """

    def __init__(self):
        smach.State.__init__(self,outcomes=["done"],
                             input_keys=["key_or_marked_text", "behaviors", 
                                         "wav_file_loc"],
                             output_keys=["text","ordered_behaviors","wav_file"])

    def execute(self,userdata):
        if userdata.behaviors != None:
            userdata.ordered_behaviors = sorted(userdata.behaviors,key = lambda b: b["start"])
        else:
            userdata.ordered_behaviors = userdata.behaviors
        userdata.text = userdata.key_or_marked_text
        userdata.wav_file = userdata.wav_file_loc

               
        rospy.logwarn("Speech debug info:")
        rospy.logwarn("Ordered Behaviors: {}".format(userdata.behaviors))
        rospy.logwarn("Text: {}".format(userdata.key_or_marked_text))
        rospy.logwarn("Wave file: {}".format(userdata.wav_file_loc))

        return "done"



class FileSpeechStart(smach.State):
    """ Speech prep from file

    State to read text, wave file, and ordered behaviors from a phrase file,
    given a key into that file.

    **Input keys**
        * key : str
            A key into the phrase file

    **Output keys**
        * text : str
            If the text is included in the phrase file, the text, otherwise
            None
        * ordered_behaviors : list of dict
            A list of behavior dictionaries.  A behavior has the keys "id",
            "start", and "args".  Ordered by start time.
        * wav_file : str
            The path to the audio file.

    **Outcomes**
        * done
            Finished fetching behaviors
        * not_found
            Unable to find the key in the phrase file
    
    """
    def __init__(self, phrases):
        """ Constructor

        Initializes TTS for speech using a phrase file.

        Parameters
        -----------

        phrases : str
            Path to the phrase file to use.
        """
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
    """ Pass through speech info without prep

    State for the case where the dialogue state machine will be given text,
    behaviors (not necessarily ordered), and a path to a wave file in
    the userdata.

    **Input keys**
        * behaviors : list of dict
             A list of behavior dictionaries.  A behavior has the keys "id",
            "start", and "args".
        * wav_file_loc : str
             The path to the audio file to play

    **Output keys**
        * text : str
            Always None
        * ordered_behaviors : list of dict
            A list of behavior dictionaries.  A behavior has the keys "id",
            "start", and "args". Ordered by start time.
        * wav_file : str
            The path to the audio file.

    **Outcomes**
        * done
            Finished fetching behaviors
        * missing_info
            Missing information in the input keys
    
    """
    def __init__(self):
        """ Constructor

        Initializes passthrough state for speech features.
        """
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

class SpeechDebugState(smach.State):
    """ Speech debug state
    
    Doesn't do anything, allowing for debugging of speech input without
    requiring tts or audio files.

    **Input keys**
        * text : str
            Can be None; the text version of the robot's speech.  Used to print
            to the screen.
        * wav_file : str
            Can be None if TTS is on. The path to the audio file.

    **Outcomes**
        * done
            Finished fetching behaviors. Always returns this.
        * no_audio
            Couldn't find the wave file, or no wave file provided and TTS
            turned off. Never returns this.
        * preempted
            State was preempted before audio finished playing.  If preempted,
            will try to stop the audio. Never returns this.


    """
    def __init__(self, synchronizer):
        
        smach.State.__init__(self,outcomes=["preempted","no_audio","done"],
                             input_keys=["text","wav_file"])
        self._sync = synchronizer

    def execute(self,userdata):
        return "done"

class SpeechState(smach.State):
    """ Speech player state

    Takes in text and/or a wave file.  Given a wave file, plays the file.
    If no wave file is provided and text-to-speech is on, fetch the audio from
    Amazon Polly and play the audio.  If no wave file is provided and text-to-
    speech is off, return "no_audio"

    **Input keys**
        * text : str
            Can be None; the text version of the robot's speech.  Used to print
            to the screen.
        * wav_file : str
            Can be None if TTS is on. The path to the audio file.

    **Outcomes**
        * done
            Finished fetching behaviors
        * no_audio
            Couldn't find the wave file, or no wave file provided and TTS
            turned off
        * preempted
            State was preempted before audio finished playing.  If preempted,
            will try to stop the audio.

    """
    def __init__(self, use_tts, synchronizer, phrases=None, voice="Kimberly"):
        """ Constructor

        Initializes the speech state with the desired parameters; either with
        or without online TTS, and with or without a pre-generated phrase
        file.

        Parameters
        -----------
        use_tts : bool
            If true, allow online TTS
        synchronizer : Synchronizer
            Synchronizer object to allow sync of speech and behaviors.  Should
            be the same object as is passed to behavior ControllerState objects
            with ``setup_sync``
        voice : str, optional
            Which Amazon Polly voice to use. Defaults to Kimberly

        phrases : str, optional
            Path to the phrase file to use. If None, require online TTS.
        """
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
    """ SmachWrapper to set up dialogue state machine 

    Given a set of controllers, this class will put together a state machine
    that allows simultaneous speech and behaviors.  You can either run the
    state machine by calling ``standalone_start`` with the appropriate userdata,
    or you can call ``get_sm`` to get a reference to the internal state machine
    and use it as a state in a larger smach state machine.  The input keys and
    outcomes of the state machine depend on the options passed to the 
    constructor; they may include:

    **Input keys**
        * key_or_marked_text : str
            If a phrase file is provided and use_tts is true, or debug is true,
            provides either a key into the phrase file or marked-up text to say
            with online TTS
        * key : str
            If a phrase file is provided and use_tts is false, provides a key
            into the phrase file.
        * marked_text : str
            If no phrase file is provided and use_tts is true, contains
            marked-up text to say with online TTS
        * behaviors : list of dict
            If no phrase file is provided and use_tts is false, or debug is true
            directly provides the behaviors for the robot to execute.  A 
            behavior has the keys "id", "start", and "args".
        * wav_file_loc : list of dict
            If no phrase file is provided and use_tts is false, or debug is true
            directly provides a path to an audio file for the robot to play

    **Outcomes**
        * done
            Successfully played the dialogue act. This is always a potential
            outcome of the state machine.
        * preempted
            The state machine was preempted.  This is always a potential outcome
            of the state machine.
        * not_found
            If not using tts, indicates that the key was not found in the phrase
            file
        * missing_info
            If neither using tts nor a phrase file, indicates that the robot was
            not provided with behaviors and an audio file in the input userdata

    """
    def __init__(self, use_tts, phrases=None, controllers=None, voice="Kimberly",debug=False):
        """ Constructor
        
        Creates a state machine according to the provided options.

        Parameters
        -----------
        use_tts : bool
            Whether or not to use online TTS from Amazon Polly

        phrases : str, optional
            Path to the phrase file to use. Defaults to None.

        controllers : list of ControllerState, optional
            The set of controllers to use for speech.  If not provided, the
            state machine will play audio but no behaviors.

        voice : str, optional
            Which Amazon Polly voice to use. Defaults to Kimberly

        debug : bool, optional
            If true, start in debug mode, with no audio or behaviors.  Defaults
            to false.
        """
        if debug:
            condition="debug"
            outcomes=["preempted","done"]
        elif use_tts and phrases!=None:
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
            if condition=="debug":
                if "SPEECH" in outcome_map:
                    return True
            
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
            if condition=="debug":
                smach.Concurrence.add("SPEECH", SpeechDebugState(self._sync),
                            remapping={"text":"text",
                                       "wav_file":"wav_file"})
            else:
                smach.Concurrence.add("SPEECH", SpeechState(use_tts, self._sync, phrases,voice),
                                      remapping={"text":"text",
                                                 "wav_file":"wav_file"})
            if controllers != None:
                for c in controllers:
                    c.setup_sync(self._sync)
                    smach.Concurrence.add(c.name, c,
                                    remapping={"ordered_behaviors":"ordered_behaviors"})
    
                

        with self._sm:
            if condition=="debug":
                smach.StateMachine.add("START",DebugSpeechStart(),
                                       transitions={"done":"DIALOGUE"},
                                       remapping={"key_or_marked_text":"key_or_marked_text",
                                                  "text":"text",
                                                  "wav_file_loc":"wav_file_loc",
                                                  "ordered_behaviors":"ordered_behaviors",
                                                  "wav_file":"wav_file"})
            elif condition=="tts_fallback":
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
        """ Returns the state machine
        
        .. warning:: Trying to run the state machine multiple times can have unexpected results.  If you need to run it multiple times in various places in your code, use the ``standalone_start`` function.

        Returns
        ---------
        smach.StateMachine
            A reference to the state machine constructed by this object.

        """
        return self._sm

    def _service_preempt_for_children(self,sm):
        sm.service_preempt()
        try:
            children = sm.get_children().values()
            for child in children:
                self._service_preempt_for_children(child) 
        except AttributeError:
            return

    def standalone_start(self, userdata_dict={}):
        """ Runs the state machine on its own.

        This function resets all states in the state machine, sets the
        userdata using the provided dictionary (be sure it matches the
        input_keys for the options you have selected), and runs the state
        machine.

        Parameters
        -----------
        userdata_dict : dict
            A dictionary of values to update the userdata.  Which values need
            to be included depends on the options you provided when creating
            this object.
        """
        self.reset()
        userdata = smach.UserData()
        for key in userdata_dict:
            userdata[key]=userdata_dict[key]
        self._sm.set_initial_state(["START"],userdata=userdata)
        self._t = threading.Thread(target=self._sm.execute)
        self._t.start()

    def reset(self):
        """ Reset the state machine

        Resets preemption in the state machine and resets the synchronizer.

        """
        self._sync.reset()
        self._service_preempt_for_children(self._sm)
        self._outcome=None

    def get_outcome(self):
        """ Returns the outcome of the state machine.
        
        After the state machine has run to completion, this will return the
        outcome.

        Returns
        -------
        str
           The outcome of the state machine.

        """
        return self._outcome

    def preempt(self):
        """ Preempt the state machine.

        Useful in the case where the state machine is running inside an action
        server
        """
        self._sm.request_preempt()
        self._t.join()

    def is_running(self):
        """ Returns whether the state machine is currently running

        Returns
        --------
        bool
            Whether or not the state machine is currently running
        """
        return self._t.isAlive()

    def get_active_states(self):
        """ Returns the currently-active states

        If the machine is running, returns all currently-active states.
        Otherwise, returns None

        Returns
        -------
        list of str
            The names of the currently-active states
        """
        if self._sm.is_running():
            return self._sm.get_active_states()
        else:
            return None
        
