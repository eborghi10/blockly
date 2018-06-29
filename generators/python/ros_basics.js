/**
 * @fileoverview Blocks for ros basic operations.
 * @author Emiliano Borghi
 */
'use strict';

goog.provide('Blockly.Python.ros_basics');
goog.require('Blockly.Python');

Blockly.Python['ros_publisher'] = function(block) {
  var topic_name = block.getFieldValue('TOPIC_NAME');
  var number_of_messages = block.getFieldValue('NUMBER_OF_MESSAGES');
  var frequency = block.getFieldValue('FREQUENCY');
  var message = Blockly.Python.valueToCode(block, 'message', Blockly.Python.ORDER_ATOMIC);
  // Splitting incoming message
  message = message.split(Blockly.Python.STRING_DELIMITER);
  // Separating message
  var msg = message.pop();
  var pkgs = message.filter(function(element, index, array) {return (index % 2 === 0);});
  var msg_types = message.filter(function(element, index, array) {return (index % 2 === 1);});

  // Check for errors
  if (pkgs.length != msg_types.length) {
    throw "Error retrieving messages to publish."
  }

  var code = ""
  code += "import sys\n"
  code += "import time\n"
  for (var i = 0; i < pkgs.length; i++) {
    code += "from " + pkgs[i] + ".msg import " + msg_types[i] + "\n"
  }
  code += "\n"
  code += "pub = rospy.Publisher('/" + topic_name + "', " + msg_types.pop() + ", queue_size=10)\n"
  code += "msg = " + msg + "\n"
  code += "i = 0\n"
  code += "rate = rospy.Rate(" + frequency +")\n"
  code += "\n"
  code += "while not rospy.is_shutdown() and i < " + number_of_messages + ":\n"
  code += "  pub.publish(msg)\n"
  code += "  rate.sleep()\n"
  code += "  i += 1\n"
  code += "time.sleep(2)\n"

  return code;
};
