The goal of this project is to develop an autonomous navigation system for the TurtleBot4
robot, leveraging prior knowledge of the environment provided in the form of a map done with
SLAM. The system should enable the robot to move from a specified starting point, which we
call the initial pose, to a target destination by defining their respective poses. The robot should
also go between cones it may encounter on his path, recognize situations of kidnapping, and
adapt to obstacles, wether they are static or moving.

During navigation, the robot must be able to:
• Plan and follow the shortest path to the destination.
• Detect, in real time, the presence of red and yellow cones placed unpredictably along the
path.
• Dynamically adapt its trajectory to always pass to the left of red cones and to the
right of yellow cones( or viceversa). This implementation in the end follows the diagram provided
rather than the written instructions, as they were contradictory. The diagram was clearer
and more reliable as it explains visually.
• Handle the presence of both static and dynamic obstacles, avoiding collisions.
• Handle the presence of static and dynamic obstacles, avoiding collisions.
• Operate robustly even in complex scenarios, such as closely spaced cones or sudden ob-
stacles.


To test the robot, it's mandatory to download the material, the map and to connect the robot to the University Network. A script file launcher.sh is then used to launch all the terminals.
