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
import actionlib
from poli_msgs.srv import LedEye, LedEyeRequest
import hlpr_dialogue_production.msg as dialogue_msgs

class LedVisemeActionServer:
    def __init__(self):
        self._as = actionlib.SimpleActionServer("HLPR_Visemes",dialogue_msgs.VisemeAction, execute_cb=self.execute_cb, auto_start=False)
        self.change_eye = rospy.ServiceProxy("/led_eye",LedEye)
        self._as.start()
        self.viseme_mapping = {"p": LedEyeRequest.FLAT,
                               "t": LedEyeRequest.FLAT,
                               "S": LedEyeRequest.FLAT,
                               "T": LedEyeRequest.FLAT,
                               "f": LedEyeRequest.FLAT,
                               "k": LedEyeRequest.BIGOPEN,
                               "i": LedEyeRequest.OPEN,
                               "r": LedEyeRequest.FLAT,
                               "s": LedEyeRequest.FLAT,
                               "u": LedEyeRequest.FLAT,
                               "@": LedEyeRequest.BIGOPEN,
                               "a": LedEyeRequest.BIGOPEN,
                               "e": LedEyeRequest.OPEN,
                               "E": LedEyeRequest.OPEN,
                               "i": LedEyeRequest.OPEN,
                               "o": LedEyeRequest.WHISTLE,
                               "O": LedEyeRequest.BIGOPEN,
                               "u": LedEyeRequest.WHISTLE,
                               "sil": LedEyeRequest.SMILE}
        self.prev_viseme = "sil"

        
    def execute_cb(self,req):
        if req.viseme != self.prev_viseme:
            self.change_eye(command=LedEyeRequest.UPDATE, which_part=LedEyeRequest.MOUTH, which_feature=LedEyeRequest.SHAPE, mouth_shape=self.viseme_mapping[req.viseme])
            self.prev_viseme = req.viseme
        self._as.set_succeeded(dialogue_msgs.VisemeResult(success=True))
            

'''{"p": "M_B_P",
    "t": "N_NG_D_Z",
    "S": "CH_SH_ZH",
    "T": "N_NG_D_Z",
    "f": "M_B_P",
    "k": "AA_AH",
    "i": "EY",
    "r": "R_ER",
    "s": "N_NG_D_Z",
    "u": "CH_SH_ZH",
    "@": "AA_AH",
    "a": "AA_AH",
    "e": "EY",
    "E": "EH_AE_AY",
    "i": "EY",
    "o": "AO_AW",
    "O": "AA_AH",
    "u": "AO_AW",
    "sil": "IDLE"}'''

        
if __name__=="__main__":
    rospy.init_node("viseme_action_server")
    l = LedVisemeActionServer()
    
