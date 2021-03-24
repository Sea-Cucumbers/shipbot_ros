#!/bin/bash
rosservice call /arm_control_node/spin_shuttlecock "{position: {x: -0.1, y: 0.6, z: 0.3}, handle_end: {x: -0.1, y: 0.6, z: 0.21826}, vertical_spin_axis: false, clockwise: false}"
