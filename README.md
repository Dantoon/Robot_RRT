# Robot_RRT

At the moment requires multirobot_stage (run-robot-pedestrian-sim and robot_costmap.launch)

When both are up simply use
<pre><code>rosrun robot_pedestrian_rrt robot_rrt_planning</code></pre>
The RRT will not start until initial position and goal is determined. Initial Pose is automatically determined with via odom.
Map goal however has to be published to /map_goal as a geometry_msg/Twist.
