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

def get_topic_type(topic_name):
  if( not topic_name[0] == '/'):
    topic_name = '/' + topic_name
  try:
    rosgraph.Master('/rostopic').getPid()
  except socket.error:
    print 'No rosmaster found'
    return None
  master = rosgraph.Master('/rostopic')
  types = [t_type for t_name, t_type in master.getTopicTypes() if t_name == topic_name]
  if types:
    tmp = types[0].split('/')
    return tmp[-1]
  return None

class SpeechListener:
  def __init__(self, commandBuffSize=10):
    self.recog_topic = "hlpr_speech_commands"

    self.msg_type = 0
    # Listen to the voice based on topic type
    if get_topic_type(self.recog_topic) == 'StampedString':
      rospy.Subscriber(self.recog_topic, StampedString, self.callback)
      self.msg_type = 1
    elif get_topic_type(self.recog_topic) == 'SpeechCommand':
      rospy.Subscriber(self.recog_topic, SpeechCommand, self.callback)
      self.msg_type = 2
    else: #elif get_topic_type(self.recog_topic) == 'String':
      rospy.Subscriber(self.recog_topic, String, self.callback)

    #mapping from keywords to commands
    rospack = rospkg.RosPack()
    kps_path = rospack.get_path('hlpr_speech_recognition') + '/data/kps.yaml'

    self.keywords_to_commands = {}
    for data in yaml.load_all(file(kps_path,'r')):
       self.keywords_to_commands[str(data['tag'])] = data['speech']

    print self.keywords_to_commands

    self._commandBuffSize = commandBuffSize
    #self.commandsQueue = deque(maxlen=self._commandBuffSize)

    self.spinning = False

    self.last_command_fresh = False

    self.last_ts = None
    self.last_string = None

  # The following function is called each time, for every message
  def callback(self, msg):

    if self.msg_type == 1:
      self.last_string = msg.keyphrase
      self.last_ts = msg.stamp
    elif self.msg_type == 2:
      self.last_string = msg.stamped_string.keyphrase
      self.last_ts = msg.data.stamped_string.stamp
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

  def get_last_command(self):
    if not self.last_command_fresh:
      return None
    self.last_command_fresh = False
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
    rospy.init_node('speech_listener', anonymous=True)
    # if shutdown, need to clean up the commands queue
    rospy.on_shutdown(self.cleanup)
    rospy.spin()

def listener():
  sl = SpeechListener()
  sl.spin() 

if __name__ == '__main__':
  listener()
  #print get_topic_type('/hlpr_speech_commands') 
  

