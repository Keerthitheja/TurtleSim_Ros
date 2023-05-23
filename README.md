# TurtleSim_Ros

## Overview
In this project we use turtlesim bot to draw the shape of an input image

**Input:** 
![.](files/input.png)
**Output:** 
![.](files/output.gif)

## Setup and Running

 * Download the project and the package ```turtlesim_art```
 * Place the package into the `src` folder of a ROS workspace ```ROS_WS/src```
 * Build the packages
    -   `colcon build`
 * Source the workspace as `source install/setup.bash`
 * Take a test image `test_image.png`
 * Run the command - `$ros2 launch turtlesim_art turtlesim_art.launch.py image_path:=test_image.png`

 This will run the turtlesim bot and start drawing the pattern

 ## TODO
 - Spawn multiple turtles to speed up the sketch
