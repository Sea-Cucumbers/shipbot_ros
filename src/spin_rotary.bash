#!/bin/bash
rosservice call /arm_control_node/spin_rotary "{position: {x: -0.1, y: 0.6, z: 0.3}, vertical_spin_axis: false, degrees: 90}"
