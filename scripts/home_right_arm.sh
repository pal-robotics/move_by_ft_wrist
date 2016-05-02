#!/bin/bash

# rostopic pub /whole_body_kinematic_controler/arm_right_7_link_goal geometry_msgs/PoseStamped "header: 
#   seq: 0
#   stamp: 
#     secs: 0
#     nsecs: 0
#   frame_id: /world
# pose: 
#   position: 
#     x: 0.2
#     y: -0.35
#     z: 0.06
#   orientation: 
#     x: 0.6851
#     y: -0.6028
#     z: 0.3696
#     w: -0.1745" --once


rostopic pub /whole_body_kinematic_controler/arm_right_tool_link_goal geometry_msgs/PoseStamped "header: 
  seq: 0
  stamp: 
    secs: 0
    nsecs: 0
  frame_id: /world
pose: 
  position: 
    x: 0.2
    y: -0.35
    z: 0.06
  orientation: 
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0" --once


rostopic pub /pose_to_follow geometry_msgs/PoseStamped "header: 
  seq: 0
  stamp: 
    secs: 0
    nsecs: 0
  frame_id: /world
pose: 
  position: 
    x: 0.2
    y: -0.35
    z: 0.06
  orientation: 
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0" --once
