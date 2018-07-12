# Robot_RRT

At the moment requires multirobot_stage and nav_bundle. All the scripts use the corridor or corridor_complex map. For the maps to work correctly they need to be copied into the map and world folder of the multirobot_stage node's source folder respectively. To properly visualize everything in rviz, I have included the corridor.rviz file in the launch folder. This needs to be copied into the multirobot_stage/launch folder.

All scripts starting with "test" are used to test the algorithm and run 31 times. To simply run it once, use the "double-bouncer" script.

To run without scripts, you need to run multirobot stage first with
<pre><code>rosrun multirobot_stage run-robot-pedestrian-sim</code></pre>
and launch the costmap with
<pre><code>roslaunch multirobot_stage robot_costmap.launch</code></pre>

Now you can start the RRT node. Run
<pre><code>rosrun robot_rrt rrt_planning</code></pre>
for the rrt,
<pre><code>rosrun robot_rrt forward_projection</code></pre>
for the forward projection,
<pre><code>rosrun robot_rrt tracker</code></pre>
for the tracker and
<pre><code>rosrun robot_rrt cmd_hold</code></pre>
to hold the sent commands.

The RRT will not start until initial position and goal is determined. Initial Pose is automatically determined with via odom.
Map goal however has to be published to /map_goal as a geometry_msg/Pose.
<pre><code>$ rostopic pub /map_goal geometry_msgs/Point -- 20 5 0</code></pre>

The tracker at the moment can only manage two pedestrians at max. If you want to visualize the RRT itself you need to uncomment the usleep function in main() (line 33) and the markerList function in generatePoint() (line 195).
