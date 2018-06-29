/**
 * @fileoverview Blocks for geometry_msgs.
 * @author Emiliano Borghi
 */
'use strict';

goog.provide('Blockly.Blocks.geometry_msgs');
goog.require('Blockly.Blocks');


/**
 * Common HSV hue for all blocks in this category.
 */
Blockly.Blocks.geometry_msgs.HUE = 260;

// https://blockly-demo.appspot.com/static/demos/blockfactory/index.html#awp8ka
Blockly.Blocks['geometry_msgs_vector3'] = {
    init: function() {
      this.appendDummyInput()
          .appendField("Vector3")
          .appendField("x")
          .appendField(new Blockly.FieldTextInput("0"), "x")
          .appendField("y")
          .appendField(new Blockly.FieldTextInput("0"), "y")
          .appendField("z")
          .appendField(new Blockly.FieldTextInput("0"), "z");
      this.setOutput(true, "geometry_msgs/Vector3");
      this.setColour(Blockly.Blocks.geometry_msgs.HUE);
   this.setTooltip("Generate a geometry_msg/Vector3 message");
   this.setHelpUrl("http://docs.ros.org/api/geometry_msgs/html/msg/Vector3.html");
    }
};

// https://blockly-demo.appspot.com/static/demos/blockfactory/index.html#zpq4ow
Blockly.Blocks['geometry_msgs_twist'] = {
    init: function() {
      this.appendDummyInput()
        .appendField("Twist");
    this.appendValueInput("linear")
        .setCheck("geometry_msgs/Vector3")
        .setAlign(Blockly.ALIGN_RIGHT)
        .appendField("linear");
    this.appendValueInput("angular")
        .setCheck("geometry_msgs/Vector3")
        .setAlign(Blockly.ALIGN_RIGHT)
        .appendField("angular");
    this.setInputsInline(false);
    this.setOutput(true, "geometry_msgs/Twist");
    this.setColour(230);
 this.setTooltip("Generate a geometry_msgs/Twist message");
 this.setHelpUrl("http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html");
    }
};
