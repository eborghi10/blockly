/**
 * @license
 *
 * Copyright 2015 Erle Robotics
 * http://erlerobotics.com
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @fileoverview Blocks for std_msgs.
 * @author Lucas Walter
 */
'use strict';

goog.provide('Blockly.Python.std_msgs');
goog.require('Blockly.Python');

Blockly.Python['std_msgs_empty'] = function(block) {
  var code = ""
  code+="import sys\n"
  code+="import time\n"
  code+="from std_msgs.msg import Empty\n"
  code+="\n"
  code+="pub = rospy.Publisher('/empty', Empty, queue_size=10)\n"
  code+="msg = Empty()\n"
  code+="i = 0\n"
  code+="rate = rospy.Rate(10)\n"
  code+="\n"
  code+="####################\n"
  code+="## EMPTY          ##\n"
  code+="####################\n"
  code+="while not rospy.is_shutdown() and i < 10:\n"
  code+="  pub.publish(msg)\n"
  code+="  rate.sleep()\n"
  code+="  i += 1\n"
  code+="time.sleep(2)\n"

  return code;
};


