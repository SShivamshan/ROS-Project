#titre Navigation system for turtlebot burger 3 in a gazebo environment


This package contains 4 nodes:
    1. The camera node
    2. The control node
    3. The lidar node
    4. The tunnel node which can be used to test the algorithm for the movement in a tunnel

This package also contains : 
    1. Two launch files with a parameter called challenge which is set to either "challenge1" or "challenge2" depending on the launch file
    2. The urdf file
    3. The worlds file
    4. The rviz file


For the first challenge, please launch the challenge1.launch, this will start the robot at the beginning of the track unitl the entrance of the tunnel. 

For the second challenge, launch the challenge2.launch, this will start the robot at the entrance of tunnel and the robot would at the entrance of the third challenge. 
