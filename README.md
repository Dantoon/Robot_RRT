# Robot_RRT

At the moment requires multirobot_stage (run-robot-pedestrian-sim and robot_costmap.launch)

When both are up simply use
<pre><code>rosrun robot_pedestrian_rrt command_sampling</code></pre>
The RRT will not start until initial position and goal is determined. Initial Pose is automatically determined with via odom.
Map goal however has to be published to /map_goal as a geometry_msg/Pose.
<pre><code>$ rostopic pub /map_goal geometry_msgs/Pose '[x,y,z]' '[x,y,z,w]'</code></pre>
At the moment only visual and generates only one path per execution.
