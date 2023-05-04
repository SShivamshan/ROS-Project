# Navigation system for turtlebot burger 3 in a gazebo environment

For the ROS and experimental robotics course, we were tasked with programming the turlebot burger 3 to drive autonomously around a circuit.The implemented algorithm should also works for both the real robot and the simulated environment.  This circuit is divided into 3 challenges. The first challenge is to follow the lane passing and avoiding the obstacles present in it's way to get to the entrance of the tunnel. The second challenge is to move through the tunnel without touching the sides of the tunnel. Finally the third challenge consist of moving through a obstructed environment and to go through two goal posts. 

This package contains 4 nodes:
* The camera node
* The control node
* The lidar node 
* The tunnel node which can be used to test the algorithm for the movement in a tunnel

This package also contains : 
* Two launch files with a parameter called challenge which is set to either "challenge1" or "challenge2" depending on the launch file
* The urdf file
* The worlds file
* The rviz file


For the first challenge, please launch the challenge1.launch, this will start the robot at the beginning of the track unitl the entrance of the tunnel. 

For the second challenge, launch the challenge2.launch, this will start the robot at the entrance of tunnel and the robot would at the entrance of the third challenge. 

**TO DO :**
* The challenge n°3 needs to be implemented the algorithm works but the robot only avoids the obstacles and doesn't turn towards the goal posts. 
* Find the transition conditions :
    * For the robot to go from the line following from challenge 1 to enter the tunnel (challenge 2)
    * After the line following from the challenge 2 to enter challenge 3
