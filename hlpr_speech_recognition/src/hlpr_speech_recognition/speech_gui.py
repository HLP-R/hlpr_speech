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
# Authors: Priyanka Khante
# Edited: Vivian Chu, 8-29-16 - rosparam and multiple yaml files
"""
speech_gui.py

Speech GUI that publishes "fake" speech commands to the
speech command topic
"""

import rospy
import sys
import rospkg
import yaml
from std_msgs.msg import String
from hlpr_speech_msgs.msg import StampedString
from PyQt4 import QtGui, QtCore
from .speech_listener import SpeechListener

class SpeechGui(QtGui.QWidget):

  def __init__(self):
      QtGui.QWidget.__init__(self)
 
      newFont = QtGui.QFont("Times", 24, QtGui.QFont.Bold)

      # Add a main layout
      mainLayout = QtGui.QVBoxLayout(self)

      # Add buttons with the commands
      grid = QtGui.QGridLayout()
      grid.setSpacing(20)

      # Initialize rosnode
      rospy.init_node("speech_gui")
   
      # Default values for speech listeners  
      rospack = rospkg.RosPack()
      default_pub_topic = 'hlpr_speech_commands'

      # Pull values from rosparam
      self.recog_topic = rospy.get_param(SpeechListener.COMMAND_TOPIC_PARAM, default_pub_topic)
      self.str_msg = rospy.get_param(SpeechListener.COMMAND_TYPE, False)

      # Wait for listener to be ready to know what commands to send
      self.service_topic = rospy.get_param(SpeechListener.SERVICE_TOPIC_PARAM, None)
      rospy.loginfo("Waiting for speech service for keywords")
      rospy.wait_for_service(self.service_topic)
      rospy.loginfo("Speech service loaded")

      # Get commands from the listener
      self.keywords = rospy.get_param(SpeechListener.KEYWORDS_PARAM, dict()).values()
      self.commands = [val for sublist in self.keywords for val in sublist]
      self.commands.sort()
 
      positions = [(i,j) for i in range(len(self.commands)) for j in range(3)]
           
      for position, name in zip(positions, self.commands):
          button = QtGui.QPushButton(name)
          button.setObjectName('%s' % name)
          button.setFont(newFont)
          button.setStyleSheet("background-color: #ccffe6")
          button.clicked.connect(self.handleButton)
          grid.addWidget(button, *position)

      mainLayout.addLayout(grid)
      mainLayout.addStretch()
      
      # Show the GUI 
      self.adjustSize()
      self.setWindowTitle("Speech Commands Interface")
      self.show()
      self.raise_()

      # Create the publisher to publish the commands to
      self.pub = rospy.Publisher(self.recog_topic, StampedString, queue_size=1)

      rospy.loginfo("Finished initializing speech GUI")
  
  # Button handler after its clicked
  def handleButton(self):
      clicked_button = self.sender()

      # Publish everytime a command is selected from the combo box
      command = str(clicked_button.objectName())
      if self.str_msg == 'String':
        self.pub.publish(command)
      else:
        keyphrase = StampedString()
        keyphrase.keyphrase = command
        keyphrase.stamp = rospy.get_rostime()
        self.pub.publish(keyphrase)

def gui_start():
    app = QtGui.QApplication(sys.argv)
    sg = SpeechGui()
    sys.exit(app.exec_())



