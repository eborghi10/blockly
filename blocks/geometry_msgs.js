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
          .appendField(new Blockly.FieldTextInput("0"), "X")
          .appendField("y")
          .appendField(new Blockly.FieldTextInput("0"), "Y")
          .appendField("z")
          .appendField(new Blockly.FieldTextInput("0"), "Z");
      this.setOutput(true, "geometry_msgs/Vector3");
      this.setColour(Blockly.Blocks.geometry_msgs.HUE);
   this.setTooltip("Generate a Vector3 geometry_msgs");
   this.setHelpUrl("http://docs.ros.org/api/geometry_msgs/html/msg/Vector3.html");
    }
};

// https://blockly-demo.appspot.com/static/demos/blockfactory/index.html#tdf77r
Blockly.Blocks['geometry_msgs_twist'] = {
    init: function() {
      this.appendDummyInput()
          .appendField("Twist")
          .appendField(new Blockly.FieldTextInput("cmd_vel"), "TOPIC_NAME")
        //   .appendField(new Blockly.FieldNumber(1, 0, Infinity, 1), "NUMBER_OF_MESSAGES")
          .appendField(new Blockly.FieldTextInput("1"), "NUMBER_OF_MESSAGES")
          .appendField("messages")
          .appendField("for")
        //   .appendField(new Blockly.FieldNumber(10, 0, 100, 1), "FREQUENCY")
          .appendField(new Blockly.FieldTextInput("10"), "FREQUENCY")
          .appendField("Hz");
      this.appendValueInput("Linear")
          .setCheck("geometry_msgs/Vector3")
          .setAlign(Blockly.ALIGN_RIGHT)
          .appendField("Linear");
      this.appendValueInput("Angular")
          .setCheck("geometry_msgs/Vector3")
          .setAlign(Blockly.ALIGN_RIGHT)
          .appendField("Angular");
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(Blockly.Blocks.geometry_msgs.HUE);
   this.setTooltip("");
   this.setHelpUrl("");
    }
};
