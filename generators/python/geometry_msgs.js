/**
 * @fileoverview Blocks for geometry_msgs.
 * @author Emiliano Borghi
 */
'use strict';

goog.provide('Blockly.Python.geometry_msgs');
goog.require('Blockly.Python');


Blockly.Python['geometry_msgs_vector3'] = function(block) {
    var x = block.getFieldValue('x');
    var y = block.getFieldValue('y');
    var z = block.getFieldValue('z');

    var code = "geometry_msgs"
    code += Blockly.Python.STR_DELIMITER
    code += "Vector3"
    code += Blockly.Python.STR_DELIMITER
    code += "Vector3(" + x + ", " + y + ", " + z + ")"

    return [code, Blockly.Python.ORDER_ATOMIC];
};

Blockly.Python['geometry_msgs_twist'] = function(block) {
  var linear_velocity = Blockly.Python.valueToCode(block, 'linear', Blockly.Python.ORDER_ATOMIC);
  var angular_velocity = Blockly.Python.valueToCode(block, 'angular', Blockly.Python.ORDER_ATOMIC);

  linear_velocity = linear_velocity.split(Blockly.Python.STR_DELIMITER).pop();
  angular_velocity = angular_velocity.split(Blockly.Python.STR_DELIMITER).pop();

  var code = "geometry_msgs"
  code += Blockly.Python.STR_DELIMITER
  code += "Vector3"
  code += Blockly.Python.STR_DELIMITER
  code += "geometry_msgs"
  code += Blockly.Python.STR_DELIMITER
  code += "Twist"
  code += Blockly.Python.STR_DELIMITER
  code+="Twist(" + linear_velocity + ", " + angular_velocity + ")"

  return [code, Blockly.Python.ORDER_ATOMIC];
};
