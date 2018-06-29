/**
 * @fileoverview Blocks for geometry_msgs.
 * @author Emiliano Borghi
 */
'use strict';

goog.provide('Blockly.Blocks.std_msgs');
goog.require('Blockly.Blocks');


/**
 * Common HSV hue for all blocks in this category.
 */
Blockly.Blocks.std_msgs.HUE = 260;

// https://blockly-demo.appspot.com/static/demos/blockfactory/index.html#thmema
Blockly.Blocks['std_msgs_bool'] = {
    init: function() {
        this.appendDummyInput()
            .appendField("Bool")
            .appendField(new Blockly.FieldDropdown([["True","True"], ["False","False"]]), "msg");
        this.setOutput(true, "std_msgs/Bool");
        this.setColour(Blockly.Blocks.std_msgs.HUE);
    this.setTooltip("Send a Bool std_msg");
    this.setHelpUrl("http://docs.ros.org/api/std_msgs/html/msg/Bool.html");
    }
};

// https://blockly-demo.appspot.com/static/demos/blockfactory/index.html#uw4ou6
Blockly.Blocks['std_msgs_empty'] = {
    init: function() {
        this.appendDummyInput()
            .appendField("Empty")
        this.setOutput(true, "std_msgs/Empty");
        this.setColour(Blockly.Blocks.std_msgs.HUE);
    this.setTooltip("Send an Empty std_msg");
    this.setHelpUrl("http://docs.ros.org/api/std_msgs/html/msg/Empty.html");
    }
};
