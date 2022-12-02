# SimplePointRobots
Basic simulation with ROS2 interfaces. A simulation instance is currently designed to represent a "team" of robots.
That is, one instance of the simulation can contain multiple robots, but they belong to the same group (namespace).


See the included launch files for configurable settings for the simulator.

### Brief Overview:   
Launch the simulation instance with:   
```ros2 launch src/freyja_simulator/freyja_simulator.launch.yaml team_namespace:=team1```

#### Parameters:    
- `robot_num_range` -- `[n1, n2]` : creates robot instances ranging from "robotn1" to "robotn2" (inclusive).   
- `team_color` -- `[r,g,b]` : colors the visualization marker.   
- `team_id` -- `<int>` : assigns an ID for the team (must be unique).
