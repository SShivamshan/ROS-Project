# Navigation system for turtlebot burger 3 in a gazebo environment

For the ROS and experimental robotics course, we were tasked with programming the turlebot burger 3 to drive autonomously around a circuit.The implemented algorithm should also works for both the real robot and the simulated environment.  This circuit is divided into 3 challenges. The first challenge is to follow the lane passing and avoiding the obstacles present in it's way to get to the entrance of the tunnel. The second challenge is to move through the tunnel without touching the sides of the tunnel. Finally the third challenge consist of moving through a obstructed environment and to go through two goal posts. 

This package contains 4 nodes:
* The camera node (pre process the data and publishes the necessary informations)
* The control node
* The lidar node 
* The tunnel node which can be used to test the algorithm for the movement in a tunnel

The **control node** contains a PID controller for both lane following and the movement in the tunnel algorithm, an algorithm for the obstacle avoidance present in the lane. This node also does the transitions to go through different challenges.  
The **camera node** is in charge of pre processing the compressed image data to retrieve both lanes and their centroids which is then published as information through the /centroid topic. This topic also contains 2 other information necessary for the lane following algorithm.    
The **lidar node** contains the pre processed data for the necessary ranges, this nodes also changes the "inf" character sent by the lidar when the robot is too close or too far away from an object. The "inf" character is changed to the max range value of the lidar. This node was supposed to publish this data to the topic /lidar_data but the data that was being sent were multiple arrays. Thus function in charge of the pre process of data were sent inside to the control node directly.   
The **tunnel node** contains the algorithm for the movement in the tunnel which can be tested separetly by placing the robot infront of the tunnel.  

This package also contains : 
* Two launch files with a parameter called challenge which is set to either "challenge1" or "challenge2" depending on the launch file
* The urdf file
* The worlds file
* The rviz file

For the first challenge, please launch the challenge1.launch, this will start the robot at the beginning of the track unitl the entrance of the tunnel. 

For the second challenge, launch the challenge2.launch, this will start the robot at the entrance of tunnel and the robot would at the entrance of the third challenge. 

**TO DO :**
* The challenge n°3 needs to be implemented, the algorithm works but the robot only avoids the obstacles and doesn't turn towards the goal posts or even goes to the goal posts.
* Find the transition conditions :
    * For the robot to go from the line following from challenge 1 to enter the tunnel (challenge 2)
    * After the line following from the challenge 2 to enter challenge 3
* The lidar node doesn't publish yet the pre processed data since the data are arrays (find a method to publish muliple arrays and how to use them once we get it).   

The course in question :   
![alt text](https://github.com/SShivamshan/ROS-Project/blob/main/COURSE.png?raw=true)  


On the real robot, we managed to finish the first and second challenge separately without any transition in between and was quite blocked for the third challenge. For the simulated robot, the robot finishes the entire first challenge and stops at the entrance of the tunnel. As for the second challenge, the robot moves around the tunnel and gets out of the tunnel and then enters the lane following algorithm and ends at the entrance of the third challenge. 
