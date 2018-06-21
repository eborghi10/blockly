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

// https://blockly-demo.appspot.com/static/demos/blockfactory/index.html#uw4ou6
Blockly.Blocks['std_msgs_empty'] = {
    init: function() {
        this.appendDummyInput()
            .appendField("Empty")
            .appendField(new Blockly.FieldTextInput("empty"), "TOPIC_NAME")
            .appendField(new Blockly.FieldTextInput("1"), "NUMBER_OF_MESSAGES")
            .appendField("messages")
            .appendField("for")
            .appendField(new Blockly.FieldTextInput("10"), "FREQUENCY")
            .appendField("Hz");
        this.setInputsInline(true);
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(Blockly.Blocks.std_msgs.HUE);
    this.setTooltip("Send an Empty std_msg");
    this.setHelpUrl("http://docs.ros.org/api/std_msgs/html/msg/Empty.html");
    }
};
