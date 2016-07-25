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
#

# TODO: Get the folder location for the txt, lm and dic files and 
# verbose, message type and recognition thresholds with ros_param

import roslib; 
import rospy
from std_msgs.msg import String
from hlpr_speech_msgs.msg import StampedString, SpeechCommand
import sys, os
from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *
import pyaudio
import rospkg

N_CHANNELS = 1
RATE = 16000
BUFFER_SIZE = 2048

RECOGNITION_THRESH = 300 #higher reduces false positives but makes it harder to detect
VERBOSE = True # prints out more info
STR_MSG = False # True if the message is only str, false if it includes a header

def SpeechRecognizer():
  modeldir = "/usr/local/share/pocketsphinx/model"

  # get an instance of RosPack with the default search paths
  rospack = rospkg.RosPack()

  # Create a decoder with certain model
  config = Decoder.default_config()
  config.set_string('-hmm', os.path.join(modeldir, 'en-us/en-us'))
  #lm_path = rospack.get_path('hlpr_speech_recognition') + '/data/kps.lm'
  #config.set_string('-lm', lm_path)
  dict_path = rospack.get_path('hlpr_speech_recognition') + '/data/kps.dic'
  config.set_string('-dict', dict_path)
  kps_path = rospack.get_path('hlpr_speech_recognition') + '/data/kps.txt'
  config.set_string('-kws', kps_path) #A file with keyphrases to spot, one per line
  config.set_float('-kws_threshold', 1e-2) #Threshold for p(hyp)/p(alternatives) ratio
  config.set_float('-kws_plp',1e-10 ) #Phone loop probability for keyword spotting
  #config.set_float('-kws_delay', 1) #Delay to wait for best detection score
  
  if not VERBOSE:
    config.set_string('-logfn','/dev/null')
  
  if STR_MSG:
    pub = rospy.Publisher('hlpr_speech_commands', String)
  else:
    pub = rospy.Publisher('hlpr_speech_commands', StampedString)

  rospy.init_node("hlpr_speech_recognizer")
  p = pyaudio.PyAudio()

  stream = p.open(format=pyaudio.paInt16,
                  channels=N_CHANNELS,
                  rate=RATE,
                  input=True,
                  frames_per_buffer=BUFFER_SIZE)
  stream.start_stream()

  # Process audio chunk by chunk. On keyword detected perform action and restart search
  decoder = Decoder(config)
  decoder.start_utt()

  while not rospy.is_shutdown():
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
      if VERBOSE:
        print ([(seg.word, seg.prob, seg.start_frame, seg.end_frame) for seg in decoder.seg()])

      if selectedSegment.prob > RECOGNITION_THRESH:
        if not hypothesis.hypstr == selectedSegment.word:
          print "Hypothesis and the selected segment do not match! Going with the selected segment"
        
        print ("Detected keyword: " + selectedSegment.word)
        # Get the time stamp for the message
        now = rospy.get_rostime()
        
        if STR_MSG:
          keyphrase = selectedSegment.word
        else:
          keyphrase = StampedString()
          keyphrase.keyphrase = selectedSegment.word
          keyphrase.stamp = rospy.get_rostime()

        pub.publish(keyphrase)
      elif VERBOSE:
        print "Not confident enough in the detected keyword"

      decoder.end_utt()
      decoder.start_utt()

if __name__ == '__main__':
  SpeechRecognizer()
   
  
