
"use strict";

let waypoints = require('./waypoints.js')
let go_to = require('./go_to.js')
let goto_command = require('./goto_command.js')
let go_to_multiple = require('./go_to_multiple.js')
let set_states = require('./set_states.js')

module.exports = {
  waypoints: waypoints,
  go_to: go_to,
  goto_command: goto_command,
  go_to_multiple: go_to_multiple,
  set_states: set_states,
};
