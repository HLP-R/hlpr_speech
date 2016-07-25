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
# Authors: Kalesha Bullard
#
# Use espeak if you do not have cepstral
#

import os
import sys
import getopt
from aenum import Enum


class Synthesizer(Enum):
  cepstral = 1
  espeak = 2

class SpeechSynthesizer(object):

  synthesizer = Synthesizer.espeak

  # CONSTRUCTOR
  def __init__(self, voice='default'):
    Synthesizer.synthesizer = Synthesizer.cepstral
    self.setVoice(voice)
    self.setRate('default')
    self.setPitch('default')
    self.setVolume('default')

  # MUTATORS
  def setVoice(self, voice):
    if(voice.lower().strip() == 'default'):
      if(Synthesizer.synthesizer == Synthesizer.cepstral):
        self.voice = 'Robin'
      else:
        self.voice = 'Ella'
    else:
      self.voice = voice

  def setRate(self, rate):
    if(rate.lower().strip() == 'default'):
      self.rate = 'default'
      if(SpeechSynthesizer.synthesizer == Synthesizer.espeak):
        self.rate = 150
    else:
      self.rate = rate
    
  def setPitch(self, pitch):
    if(pitch.lower().strip() == 'default'):
      self.pitch = 'default'
      if(SpeechSynthesizer.synthesizer == Synthesizer.espeak):
        self.pitch = 55
    else:
      self.pitch = pitch
    
  def setVolume(self, vol):
    if(SpeechSynthesizer.synthesizer == Synthesizer.cepstral and 
       vol.lower().strip() == 'default'):
      self.volume = 'default'

  @staticmethod
  def setSynthesizer(system):
    if(system.lower().strip() == 'cepstral'):
      SpeechSynthesizer.synthesizer = Synthesizer.cepstral
    elif(system.lower().strip() == 'espeak'):
      SpeechSynthesizer.synthesizer = Synthesizer.espeak


  # OTHER FUNCTIONS
  def say(self, message, rate='default', pitch='default', volume='default'):
    print "\nBelow is the message to be synthesized into speech:"

    self.setRate(rate)
    self.setPitch(pitch)
    self.setVolume(volume)
    
    speech_command = None
    if(SpeechSynthesizer.synthesizer == Synthesizer.cepstral):
      speech_command = "padsp swift -n " + self.voice + " \"" + message + "\""
    elif(SpeechSynthesizer.synthesizer == Synthesizer.espeak):
      speech_command = "espeak\t-s\t" + str(self.rate) + "\t-p\t" + str(self.pitch) + "\t\"" + message + "\"\t2>/dev/null"
    
    if(speech_command != None):
      print speech_command
      os.system(speech_command)

  def outputToFile(self, message, filename):
    output_message = "\nThe message: \"" + message + "\"" + " will be saved to " + filename + "."
    print(output_message)


  def usage(self):
    print "\npython args_parser.py [option] ... [-s  --say | -o  --output | ...] [arg] ...\n"
    print "Options and Arguments:"
    print "-h  --help\t: print this help message and exit"
    print "-o  --output\t: output to file, use with arg = filename.wav"
    print "-p  --pitch\t: set pitch of voice"
    print "-r  --rate\t: set rate of voice"
    print "-s  --say\t: provide string message to synthesized into speech"
    print "-v  --voice\t: set voice to be used for speech synthesis"
    print "\n"

def main():
  #print "\nCommand Line Args:"
  #print str(sys.argv[1:])

  #SpeechSynthesizer.setSynthesizer('espeak')
  
  #synthesizer = SpeechSynthesizer('Robin')
  synthesizer = SpeechSynthesizer()
  
  try:
    opts, args = getopt.getopt(sys.argv[1:], 'ho:', ["help", "say=", "output="])
  except getopt.GetoptError as err:
    # Print help information and exit
    print str(err)
    synthesizer.usage()
    sys.exit(2)

  message = None
  filename = None
  for o, a in opts:
    if o in ("-h", "--help"):
      synthesizer.usage()
      sys.exit()
    elif o in ("-s", "--say"):
      message = a
      synthesizer.say(message)
    elif o in ("-o", "--output"):
      filename = a
    else:
      assert False, "unhandled option"

  if(filename != None):
    synthesizer.outputToFile(message, filename)
  print("\n")

if __name__ == '__main__':
  main()
