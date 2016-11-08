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
# Authors: Baris Akgun, Priyanka Khante
# Edited: Vivian Chu, 8-29-16 - rosparam and multiple yaml files
#
# A convenience class to map speech recognition result to commands 
# while keeping the time stamp.
#
# Note that currently the mapping is done by hand

import rospy
import rosgraph
import rospkg
import socket
import yaml
from std_msgs.msg import String
from hlpr_speech_msgs.msg import StampedString, SpeechCommand
from hlpr_speech_msgs.srv import SpeechService

class SpeechListener:

  COMMAND_TOPIC_PARAM = "/speech/publish_topic"
  SERVICE_TOPIC_PARAM = "/speech/service_topic"
  KEYWORDS_PARAM = "/speech/keywords"
  COMMAND_TYPE = "/speech/command_type"
  LEAVE_COMMAND = "/speech/leave_command"

  def __init__(self, commandBuffSize=10, init_node=True):

    if (init_node):
      # initialize the ros node
      rospy.init_node("speech_listener")
  
    # Default values for speech listener
    rospack = rospkg.RosPack()
    default_pub_topic = 'hlpr_speech_commands'
    default_yaml_files = [rospack.get_path('hlpr_speech_recognition')+'/data/kps.yaml']
    default_service_topic = 'get_last_speech_cmd'

    # Pull values from rosparam
    self.recog_topic = rospy.get_param(SpeechListener.COMMAND_TOPIC_PARAM, default_pub_topic)
    self.yaml_files = rospy.get_param("~yaml_list", default_yaml_files)
    self.service_topic = rospy.get_param(SpeechListener.SERVICE_TOPIC_PARAM, default_service_topic)
    self.msg_type = eval(rospy.get_param(SpeechListener.COMMAND_TYPE, 'StampedString')) # True if message is only str, false includes header
    self.leave_command_flag = rospy.get_param(SpeechListener.LEAVE_COMMAND, False) #do we care if we the last command is old

    rospy.Subscriber(self.recog_topic, self.msg_type, self.callback)

    # Converts the yaml files into keywords to store into the dictionary
    self.keywords_to_commands = {}
    for kps_path in self.yaml_files:
       for data in yaml.load_all(file(kps_path,'r')):
          self.keywords_to_commands[str(data['tag'])] = data['speech']

    # Store this on the rosparam server now
    rospy.set_param(SpeechListener.KEYWORDS_PARAM, self.keywords_to_commands)

    self._commandBuffSize = commandBuffSize
    #self.commandsQueue = deque(maxlen=self._commandBuffSize)

    # Flags for starting/stopping the node
    self.spinning = False
    self.last_command_fresh = False
    self.last_command = None
    self.last_ts = None
    self.last_string = None

    # Setup service call
    s = rospy.Service(self.service_topic, SpeechService, self.get_last_command)
    rospy.loginfo("Speech listener initialized")

  # The following function is called each time, for every message
  def callback(self, msg):

    if self.msg_type == StampedString:
      self.last_string = msg.keyphrase
      self.last_ts = msg.stamp
    else:
      self.last_string = msg.data

    self.last_command = self._map_keyword_to_command(self.last_string)
    self.last_command_fresh = True
    if self.spinning:
      rospy.loginfo(rospy.get_caller_id() + '  I heard %s', str(self.last_command))

  # method to extract command string from msg
  def _map_keyword_to_command(self, data):
    for (command, keywords) in self.keywords_to_commands.iteritems():
      for word in keywords:
        if data.find(word) > -1:
          return command

  # This is now made a service call
  def get_last_command(self, req=None):

    # Check if we care how "recent" the command was
    if not self.leave_command_flag:

      # returns a service request error
      if not self.last_command_fresh:
        return None

    # The command hasn't been ask for before
    self.last_command_fresh = False
    if (req): 
      return {'speech_cmd': self.last_command}
    else:
      return self.last_command

  def get_last_string(self):
    return self.last_string

  def get_last_ts(self):
    return self.last_ts
    
  # clears commands queue
  def cleanup(self):
    #commandsQueue.clear()
    pass

  def spin(self):
    self.spinning = True
    # if shutdown, need to clean up the commands queue
    rospy.on_shutdown(self.cleanup)
    rospy.spin()

def listener():
  sl = SpeechListener()
  sl.spin() 

if __name__ == '__main__':
  listener()
  
