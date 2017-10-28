# Simulator node

This ROS node simulates the movements and sensor reading of the AUV as it navigates the arena.

The node launches a pygame window to show the AUV and the landmarks

Use `W` `A` `S` `D` to control the AUV to go forward, back, left, and right

Use `Q` `E` to rotate

### Running the node

First use `catkin_make` to build the node

`rosrun simulator node.py`

### Publishes

* /landmark_observations - An array of landmarks, for details see msg folder

### Subscribes