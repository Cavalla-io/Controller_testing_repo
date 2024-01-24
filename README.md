# This is a template controller usable for any vehicle with 4 wheels

## Getting started
1. clone the repo with `git clone https://github.com/Cavalla-io/Controller_testing_repo`
2. build the repo with `colcon build` **Make sure to do this in the parent repository** it should look like `/path/to/local/repository/Controller_testing_repo$`

## Launching a robot
This will show you how to launch the basic mobile robot to learn how the controller works
1. run `ros2 launch gazebo_simulation robot_sim.launch.py` in the terminal
2. in a new terminal run `ros2 launch four_ws_control four_ws_control.launch.py`
3. in another terminal run `ros2 launch four_ws_control four_ws_pub.launch.py`
   
**Make sure to run `source install/setup.bash` in every new terminal you set up**
