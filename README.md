# Robot_RRT

At the moment requires multirobot_stage (run-robot-pedestrian-sim and robot_costmap.launch)

When both are up simply use
<pre><code>rosrun robot_rrt rrt_planning</code></pre>
for the rrt and
<pre><code>rosrun robot_rrt forward_projection</code></pre>
for the forward projection.
The RRT will not start until initial position and goal is determined. Initial Pose is automatically determined with via odom.
Map goal however has to be published to /map_goal as a geometry_msg/Pose.
<pre><code>$ rostopic pub /map_goal geometry_msgs/Point -- 20 5 0</code></pre>
At the moment only visual and generates only one path per execution.

Right now the only things visualized are the start and goal points, and the path points calculated by the forward projection.
Visualization is sent to /treepoints as markers.
