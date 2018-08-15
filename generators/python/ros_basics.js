/**
 * @fileoverview Blocks for ros basic operations.
 * @author Emiliano Borghi
 */
'use strict';

goog.provide('Blockly.Python.ros_basics');
goog.require('Blockly.Python');

Blockly.Python['ros_node'] = function(block) {
  var node_name = block.getFieldValue('node_name');
  var statements_raw = Blockly.Python.statementToCode(block, 'ros_node');
  var statements = statements_raw.split(Blockly.Python.STR_DELIMITER);

  var libs = statements.filter(function(element, index) {return (index % 3 === 1 && element.length > 1);});
  var setups = statements.filter(function(element, index) {return (index % 3 === 2 && element.length > 1);});
  var usages = statements.filter(function(element, index) {return (index % 3 === 0 && element.length > 1);});

  var code = "import sys\n"
  code += "import time\n"
  code += "import rospy\n"
  for (var i = 0; i < libs.length; i++) {
    code += libs[i].replace(/^\s+/g, '').split(Blockly.Python.STR_NEW_LINE).join('\n').split(Blockly.Python.STR_TAB).join('  ');
  }
  code += "\n"
  // code += "rospy.init_node('" + node_name + "', anonymous=True)\n"
  for (var i = 0; i < setups.length; i++) {
    code += setups[i].replace(/^\s+/g, '').split(Blockly.Python.STR_NEW_LINE).join('\n').split(Blockly.Python.STR_TAB).join('  ');
  }
  code += "\n"
  code += "while not rospy.is_shutdown():\n"
  for (var i = 0; i < usages.length; i++) {
    code += '  ' + usages[i].replace(/^\s+/g, '').split(Blockly.Python.STR_NEW_LINE).join('\n  ').split(Blockly.Python.STR_TAB).join('  ');
  }
  // code += "time.sleep(2)\n"

  return code;
};

function processROSMessages(message) {
  // Splitting incoming message
  message = message.split(Blockly.Python.STR_DELIMITER);

  // Separating message
  var msg = message.pop();
  var pkgs = message.filter(function(element, index) {return (index % 2 === 0);});
  var msg_types = message.filter(function(element, index) {return (index % 2 === 1);});

  // Check for errors
  if (pkgs.length != msg_types.length) {
    throw "Error retrieving messages to publish."
  }
  return [msg, pkgs, msg_types];
}

Blockly.Python['ros_publisher'] = function(block) {
  var topic_name_raw = block.getFieldValue('TOPIC_NAME');
  var number_of_messages = block.getFieldValue('NUMBER_OF_MESSAGES');
  var frequency = block.getFieldValue('FREQUENCY');
  var message = Blockly.Python.valueToCode(block, 'message', Blockly.Python.ORDER_ATOMIC);

  // Get ROS message
  message = processROSMessages(message);
  var msg = message[0];
  var pkgs = message[1];
  var msg_types = message[2];

  var libs = ""
  for (var i = 0; i < pkgs.length; i++) {
    libs += "from " + pkgs[i] + ".msg import " + msg_types[i] + Blockly.Python.STR_NEW_LINE
  }

  // Use a renamed "topic_name_raw"
  var topic_name = topic_name_raw.replace("/", "_");
  // The variables have appended the "topic_name" in order to use, in the same
  // code, multiple instances of the publisher.
  var pub_var = "pub_" + topic_name;
  var msg_var = "msg_" + topic_name;
  var i_var = "i_" + topic_name;
  var rate_var = "rate_" + topic_name;

  var setup = pub_var + " = rospy.Publisher('/" + topic_name_raw + "', " + msg_types.pop() + ", queue_size=10)" + Blockly.Python.STR_NEW_LINE
  setup += msg_var + " = " + msg + Blockly.Python.STR_NEW_LINE
  setup += i_var + " = 0" + Blockly.Python.STR_NEW_LINE
  setup += rate_var + " = rospy.Rate(" + frequency +")" + Blockly.Python.STR_NEW_LINE
  setup += Blockly.Python.STR_DELIMITER;

  var usage = "if " + i_var + " < " + number_of_messages + ":" + Blockly.Python.STR_NEW_LINE
  usage += Blockly.Python.STR_TAB + pub_var + ".publish(" + msg_var + ")" + Blockly.Python.STR_NEW_LINE
  usage += Blockly.Python.STR_TAB + rate_var + ".sleep()" + Blockly.Python.STR_NEW_LINE
  usage += Blockly.Python.STR_TAB + i_var + " += 1" + Blockly.Python.STR_NEW_LINE

  var code = [usage, libs, setup].join(Blockly.Python.STR_DELIMITER);
  return code;
};

Blockly.Python['ros_subscriber'] = function(block) {
  var topic_name_raw = block.getFieldValue('topic_name');
  var message = Blockly.Python.valueToCode(block, 'message', Blockly.Python.ORDER_ATOMIC);

  // Get ROS message
  message = processROSMessages(message);
  var msg = message[0];
  var pkgs = message[1];
  var msg_types = message[2];

  // Use a renamed "topic_name_raw"
  var topic_name = topic_name_raw.replace("/", "_");
  // Generate variable names
  var callback_var = "callback_" + topic_name;
  var msg_var = "msg_" + topic_name;

  var libs = ""
  for (var i = 0; i < pkgs.length; i++) {
    libs += "from " + pkgs[i] + ".msg import " + msg_types[i] + Blockly.Python.STR_NEW_LINE
  }
  libs += Blockly.Python.STR_NEW_LINE
  libs += "def " + callback_var + "(msg):" + Blockly.Python.STR_NEW_LINE
  libs += Blockly.Python.STR_TAB + msg_var + " = msg" + Blockly.Python.STR_NEW_LINE

  var setup = "rospy.Subscriber('" + topic_name_raw + "', " + msg_types.pop() + ", " + callback_var + ")" + Blockly.Python.STR_NEW_LINE;
  setup += Blockly.Python.STR_DELIMITER;

  var usage = msg_var + Blockly.Python.STR_NEW_LINE

  var code = [usage, libs, setup].join(Blockly.Python.STR_DELIMITER);

  return [code, Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python['ros_logging'] = function(block) {
  var verbosity = block.getFieldValue('verbosity');
  var message = Blockly.Python.valueToCode(block, 'message', Blockly.Python.ORDER_ATOMIC);

  var code = "rospy.log"
  switch (verbosity) {
    case 'debug':
      code += "debug"
      break;
    case 'info':
      code += "info"
      break;
    case 'warning':
      code += "warn"
      break;
    case 'error':
      code += "err"
      break;
    case 'fatal':
      code += "fatal"
      break;
  }
  code += "(" + message + ")\n"
  code += Blockly.Python.STR_DELIMITER

  return [code, Blockly.Python.ORDER_ATOMIC];
};
