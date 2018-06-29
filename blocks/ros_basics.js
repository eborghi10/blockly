/**
 * @fileoverview Blocks for ros basic operations.
 * @author Emiliano Borghi
 */
'use strict';

goog.provide('Blockly.Blocks.ros_basics');
goog.require('Blockly.Blocks');


/**
 * Common HSV hue for all blocks in this category.
 */
Blockly.Blocks.ros_basics.HUE = 170;

// https://blockly-demo.appspot.com/static/demos/blockfactory/index.html#b4jtdg
Blockly.Blocks['ros_publisher'] = {
    init: function() {
      this.appendDummyInput()
      .appendField("Publish")
      .appendField(new Blockly.FieldTextInput("topic_name"), "TOPIC_NAME")
      .appendField(new Blockly.FieldTextInput("1"), "NUMBER_OF_MESSAGES")
      .appendField("messages")
      .appendField("for")
      .appendField(new Blockly.FieldTextInput("1"), "FREQUENCY")
      .appendField("Hz");
  this.appendValueInput("message")
      .setCheck(["std_msgs/Bool", "std_msgs/Empty", "geometry_msgs/Twist", "geometry_msgs/Vector3"]);
  this.setInputsInline(true);
  this.setPreviousStatement(true, null);
  this.setNextStatement(true, null);
  this.setColour(Blockly.Blocks.ros_basics.HUE);
this.setTooltip("Send an Empty std_msg");
this.setHelpUrl("http://docs.ros.org/api/std_msgs/html/msg/Empty.html");
    }
};
