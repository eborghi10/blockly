/**
 * @fileoverview Blocks for geometry_msgs.
 * @author Emiliano Borghi
 */
'use strict';

goog.provide('Blockly.Python.geometry_msgs');
goog.require('Blockly.Python');


Blockly.Python['geometry_msgs_vector3'] = function(block) {
    var x = parseFloat(block.getFieldValue('X'));
    var y = parseFloat(block.getFieldValue('Y'));
    var z = parseFloat(block.getFieldValue('Z'));

    var code = new Array(x, y, z);
    code = code.join(', ');
    return [code, Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python['geometry_msgs_twist'] = function(block) {
  var topic_name = block.getFieldValue('TOPIC_NAME');
  var number_of_messages = block.getFieldValue('NUMBER_OF_MESSAGES');
  var frequency = block.getFieldValue('FREQUENCY');
  var linear_velocity = Blockly.Python.valueToCode(block, 'Linear', Blockly.Python.ORDER_ATOMIC);
  var angular_velocity = Blockly.Python.valueToCode(block, 'Angular', Blockly.Python.ORDER_ATOMIC);

  var code = ""
  code+="import sys\n"
  code+="import time\n"
  code+="from geometry_msgs.msg import Twist, Vector3\n"
  code+="\n"
  code+="pub = rospy.Publisher('/" + topic_name + "', Twist, queue_size=10)\n"
  code+="msg = Twist()\n"
  code+="msg.linear = Vector3(" + linear_velocity + ")\n"
  code+="msg.angular = Vector3(" + angular_velocity + ")\n"
  code+="i = 0\n"
  code+="rate = rospy.Rate(" + frequency +")\n"
  code+="\n"
  code+="while not rospy.is_shutdown() and i < " + number_of_messages + ":\n"
  code+="  pub.publish(msg)\n"
  code+="  rate.sleep()\n"
  code+="  i += 1\n"
  code+="time.sleep(2)\n"
  
  return code;
};
