# Ant_Colony_Optimizer_move_base_planner
ROS based global path planner using Ant Colony Optimization

### Not yet working. 
The map provided my the map_server is not loading into the OccupancyGrid map object used in src/ACO.cpp and elsewhere. Feel free to fix the issues.
</br>Principle working of an Ant Colony Optimizer is coded in /src/ACO.cpp.

### Basic Instructions:
* Place the "aco_ros" package in the source "src" folder for of your workspace.
* Use "catkin_make" to build or "catkin_make --pkg aco_ros" to build this package individually.
* Create your own launch file to launch your robot. Sample launch file "aco_test.launch" is given in launch folder.
* Make sure to specifiy a pre-built map to use for navigation, before the following line:</br>
   `<node name="map_server" pkg="map_server" type="map_server" args="$(arg map_file)" />`
* Mention the custom move_base launch file to use with your robot.</br>
   `<include file="$(find aco_ros)/launch/move_base_custom.launch"/>`
* If "move_base.launch" file is created separately, use the following line to access the aco_ros package as the global planner.</br>
   `<param name="base_global_planner" value="Aco_planner/AcoPlanner" />`
* In the following `<rosparam file="$(find..>` , make sure to point the params to the costmap params file in your robot setup.
  
 
### Credits: 
iPath: A C++ Library of Intelligent Global Path Planners for Mobile Robots with ROS Integration. </br>
https://github.com/coins-lab/ipath

## Related Papers:
* [Inverted Ant Colony Optimization for Search and Rescue in an Unknown Maze-like Indoor Environment](https://www.researchgate.net/profile/Dymitr_Ruta/publication/326239231_Inverted_ant_colony_optimization_for_search_and_rescue_in_an_unknown_maze-like_indoor_environment/links/5bc46fbc458515a7a9e7b78c/Inverted-ant-colony-optimization-for-search-and-rescue-in-an-unknown-maze-like-indoor-environment.pdf)
* [Genetic Programming: Evolution of Mona Lisa](https://rogerjohansson.blog/2008/12/07/genetic-programming-evolution-of-mona-lisa/)
