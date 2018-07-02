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

// https://blockly-demo.appspot.com/static/demos/blockfactory/index.html#tutziv
Blockly.Blocks['ros_node'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("ROS node");
    this.appendStatementInput("ros_node")
        .setCheck(null)
        .appendField(new Blockly.FieldTextInput("node_name"), "node_name");
    this.setInputsInline(false);
    this.setColour(230);
 this.setTooltip("Creates a ROS node");
 this.setHelpUrl("http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown#Initializing_your_ROS_Node");
  }
};

// https://blockly-demo.appspot.com/static/demos/blockfactory/index.html#8gtiv2
Blockly.Blocks['ros_publisher'] = {
    init: function() {
      this.appendValueInput("message")
        .setCheck(["std_msgs/Bool", "std_msgs/Empty", "geometry_msgs/Twist", "geometry_msgs/Vector3"])
        .appendField("Publish")
        .appendField(new Blockly.FieldTextInput("topic_name"), "TOPIC_NAME")
        .appendField(new Blockly.FieldTextInput("10"), "NUMBER_OF_MESSAGES")
        .appendField("messages")
        .appendField("at")
        .appendField(new Blockly.FieldTextInput("1"), "FREQUENCY")
        .appendField("Hz");
    this.setInputsInline(false);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
  this.setColour(Blockly.Blocks.ros_basics.HUE);
this.setTooltip("Send an Empty std_msg");
this.setHelpUrl("http://docs.ros.org/api/std_msgs/html/msg/Empty.html");
    }
};

// https://blockly-demo.appspot.com/static/demos/blockfactory/index.html#rw76zq
Blockly.Blocks['ros_subscriber'] = {
  init: function() {
    this.appendValueInput("message")
        .setCheck(["std_msgs/Bool", "std_msgs/Empty", "geometry_msgs/Twist", "geometry_msgs/Vector3"])
        .appendField("Subscribe")
        .appendField(new Blockly.FieldTextInput("topic_name"), "topic_name");
    this.setOutput(true, null);
    this.setColour(Blockly.Blocks.ros_basics.HUE);
 this.setTooltip("Creates a ROS subscriber");
 this.setHelpUrl("http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29#rospy_tutorials.2BAC8-Tutorials.2BAC8-WritingPublisherSubscriber.Writing_the_Subscriber_Node");
  }
};

// https://blockly-demo.appspot.com/static/demos/blockfactory/index.html#vd8fqt
Blockly.Blocks['ros_logging'] = {
  init: function() {
    this.appendValueInput("message")
        .setCheck(null)
        .appendField("Log")
        .appendField(new Blockly.FieldDropdown([["debug","debug"], ["info","info"], ["warning","warning"], ["error","error"], ["fatal","fatal"]]), "verbosity");
    this.setInputsInline(false);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(Blockly.Blocks.ros_basics.HUE);
 this.setTooltip("Creates a logging mechanism");
 this.setHelpUrl("http://wiki.ros.org/rospy/Overview/Logging");
  }
};
