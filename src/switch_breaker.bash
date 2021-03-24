#!/bin/bash
rosservice call /arm_control_node/switch_breaker "{position: {x: -0.1, y: 0.6, z: 0.3}, push_up: true}"
