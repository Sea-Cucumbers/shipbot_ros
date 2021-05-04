#!/bin/bash
rosservice call /chassis_control_node/travel_ol "{delta_x: -0.2, delta_y: -0.2, delta_theta: 0.785}"
