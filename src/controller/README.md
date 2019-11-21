# controller
In order to test the controller you need to clone also the simulator repo into your catkin workspace.
Check all the topics names and types.
Build your workspace
<code>
  $ catkin_make 
</code>
Start the simulation
<code>
  $ rosrun vehicle_simulation simulator.py   
</code>
 
Run the path publisher. A node as to be made for this.
 
 
Start the controller
<code>
  $ rosrun controller pure_pursuit.py   
</code>


If you open rviz and you choose the right topics (Path and Odometry) you should be able to run all the pipeline!

Main node *pure_pursuit*e.
