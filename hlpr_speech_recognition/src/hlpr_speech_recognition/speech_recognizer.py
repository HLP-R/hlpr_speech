#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2016, HLP-R
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#  notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#  copyright notice, this list of conditions and the following
#  disclaimer in the documentation and/or other materials provided
#  with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#  contributors may be used to endorse or promote products derived
#  from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# A script to use pocketsphinx's "keyphrase spotting" feature with 
# python and ros. Note that it
#
# Authors: Baris Akgun 
# Edited: Vivian Chu, 8-29-16: rosparam config values

import rospy
from std_msgs.msg import String
from hlpr_speech_msgs.msg import StampedString, SpeechCommand
import sys, os
from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *
import pyaudio
import rospkg
from .speech_listener import SpeechListener

# Global values specific to speech
N_CHANNELS = 1
RATE = 16000
BUFFER_SIZE = 6144

class SpeechRecognizer():

  def __init__(self):

    # Intialize the node
    rospy.init_node("hlpr_speech_recognizer")

    # get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()

    # Default data files for speech dictionaries
    default_modeldir = "/usr/local/share/pocketsphinx/model"
    default_dict_path = rospack.get_path('hlpr_speech_recognition') + '/data/kps.dic'
    default_kps_path = rospack.get_path('hlpr_speech_recognition') + '/data/kps.txt'
    default_rec_thresh = 300  #higher reduces false positives but makes it harder to detect
    default_pub_topic = 'hlpr_speech_commands'

    # Load model and dictionary values from param server
    modeldir = rospy.get_param("~model_dir", default_modeldir)
    dict_path = rospy.get_param("~dict_path", default_dict_path)
    kps_path = rospy.get_param("~kps_path", default_kps_path)
    self.verbose = rospy.get_param("/speech/verbose", True) # default prints out more info
    self.str_msg = rospy.get_param(SpeechListener.COMMAND_TYPE, 'StampedString') # True if message is only str, false includes header
    self.cmd_pub_topic = rospy.get_param(SpeechListener.COMMAND_TOPIC_PARAM, default_pub_topic)
   
    # Parameters for recognition
    self.RECOGNITION_THRESHOLD = rospy.get_param("/speech/rec_thresh", default_rec_thresh)

    # Create a decoder with certain model
    self.config = Decoder.default_config()
    self.config.set_string('-hmm', os.path.join(modeldir, 'en-us/en-us'))

    # Configure the dictionary - not used?
    #lm_path = rospack.get_path('hlpr_speech_recognition') + '/data/kps.lm'
    #self.config.set_string('-lm', lm_path)

    # Configuration settings for speech detection
    self.config.set_string('-dict', dict_path)
    self.config.set_string('-kws', kps_path) #A file with keyphrases to spot, one per line
    self.config.set_float('-kws_threshold', 1e-2) #Threshold for p(hyp)/p(alternatives) ratio
    self.config.set_float('-kws_plp',1e-10 ) #Phone loop probability for keyword spotting
    #self.config.set_float('-kws_delay', 1) #Delay to wait for best detection score
   
    # Check if we dump extra information to null 
    if not self.verbose:
      self.config.set_string('-logfn','/dev/null')
   
    # Setup the publisher 
    if self.str_msg == 'String':
      self.pub = rospy.Publisher(self.cmd_pub_topic, String, queue_size=1)
    else:
      self.pub = rospy.Publisher(self.cmd_pub_topic, StampedString, queue_size=1)

    rospy.loginfo("Finished initializing speech recognizer")

    # Start recognizing
    self.begin_rec()

  def begin_rec(self):

    p = pyaudio.PyAudio()
    stream = p.open(format=pyaudio.paInt16,
                    channels=N_CHANNELS,
                    rate=RATE,
                    input=True,
                    frames_per_buffer=BUFFER_SIZE)
    stream.start_stream()

    # Process audio chunk by chunk. On keyword detected perform action and restart search
    decoder = Decoder(self.config)
    decoder.start_utt()

    while not rospy.is_shutdown():
      selectedSegment = None
      buf = stream.read(BUFFER_SIZE)
      if buf:
        decoder.process_raw(buf, False, False)
      else:
        break
      if decoder.hyp() != None:
        hypothesis = decoder.hyp()
        maxProb = 0
        for seg in decoder.seg():
          if seg.prob > maxProb:
            selectedSegment = seg
            maxProb = seg.prob
        if self.verbose:
          print ([(seg.word, seg.prob, seg.start_frame, seg.end_frame) for seg in decoder.seg()])

        if selectedSegment:
          if selectedSegment.prob > self.RECOGNITION_THRESHOLD:
            if not hypothesis.hypstr == selectedSegment.word:
              print "Hypothesis and the selected segment do not match! Going with the selected segment"
            
            print ("Detected keyword: " + selectedSegment.word)
            # Get the time stamp for the message
            now = rospy.get_rostime()
            
            if self.str_msg == 'String':
              keyphrase = selectedSegment.word
            else:
              keyphrase = StampedString()
              keyphrase.keyphrase = selectedSegment.word
              keyphrase.stamp = rospy.get_rostime()

            self.pub.publish(keyphrase)
          elif self.verbose:
            print "Not confident enough in the detected keyword"
        else:
          print 'No Selected Segment'

        decoder.end_utt()
        decoder.start_utt()

if __name__ == '__main__':
  SpeechRecognizer()
   
  
