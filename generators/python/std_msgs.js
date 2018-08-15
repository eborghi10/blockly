/**
 * @fileoverview Blocks for geometry_msgs.
 * @author Emiliano Borghi
 */
'use strict';

goog.provide('Blockly.Python.std_msgs');
goog.require('Blockly.Python');

Blockly.Python['std_msgs_bool'] = function(block) {
  var dropdown_msg = block.getFieldValue('msg');

  var code = "std_msgs"
  code += Blockly.Python.STR_DELIMITER
  code += "Bool"
  code += Blockly.Python.STR_DELIMITER
  code += "Bool("+ dropdown_msg +")"

  return [code, Blockly.Python.ORDER_ATOMIC];
}

Blockly.Python['std_msgs_empty'] = function(block) {
  var code = "std_msgs"
  code += Blockly.Python.STR_DELIMITER
  code += "Empty"
  code += Blockly.Python.STR_DELIMITER
  code+="Empty()"

  return [code, Blockly.Python.ORDER_ATOMIC];
};
