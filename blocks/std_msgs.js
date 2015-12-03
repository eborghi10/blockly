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
 * @author victor@erlerobot.com (Víctor Mayoral Vilches)
 */
'use strict';

goog.provide('Blockly.Blocks.std_msgs');
goog.require('Blockly.Blocks');


/**
 * Common HSV hue for all blocks in this category.
 */
Blockly.Blocks.std_msgs.HUE = 260;


Blockly.Blocks['std_msgs_empty'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("Empty");
    this.setPreviousStatement(true);
    this.setNextStatement(true);
    this.setColour(260);
    this.setTooltip('Publish an Empty');
    this.setHelpUrl('http://lucasw.github.io/');
  }
};

