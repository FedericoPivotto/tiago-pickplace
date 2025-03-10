# GROUP 15
Federico Pivotto, 2121720, federico.pivotto@studenti.unipd.it

Fabrizio Genilotti, 2119281, fabrizio.genilotti@studenti.unipd.it

Francesco Boscolo Meneguolo, 2119969, francesco.boscolomeneguolo@studenti.unipd.it

# Simulation commands

## Build procedure
After creating and building the catkin workspace following the instructions provided in the assignment, clone the package `ir2425_group_15` (branch `master`) into the `src/` folder of the catkin workspace, then execute the `catkin build` command to build the workspace with the package.

**Note**: the build might require to be built multiple times by alternating `catkin build` and `source ./devel/setup.bash`.

## Simulation run

### World setup

#### Option 1
To execute the simulation, run the following four commands in sequence:

1. **Start the simulation** (cmd console 1):
```bash
roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=iaslab_assignment2
```

2. **AprilTag** (cmd console 2):
```bash
roslaunch tiago_iaslab_simulation apriltag2.launch
```

3. **Navigation stack** (cmd console 3): 
```bash
roslaunch tiago_iaslab_simulation navigation.launch
```

4. **Straight line server** (cmd console 4):
```bash
rosrun tiago_iaslab_simulation get_straightline_node
```

#### Option 2
To execute the simulation, run the following command:
```bash
roslaunch ir2425_group_15 env.launch
```

### Task execution
Once all the previous commands are running properly, run the following one:
```bash
roslaunch ir2425_group_15 task.launch
```


### Visualization
It is possible to visualize the simulation also through the RViz visualization tool:
```bash
roslaunch ir2425_group_15 rviz_pp.launch
```

# Package organization

## Nodes
The nodes in the package are organized into separate folders, each containing the code for the node and its implemented libraries. These nodes are:

- `start_task_node`: it has the role of the node A. It sends to `TaskStatusAction` server both the path of waypoints to reach tables area, the AprilTags IDs corresponding to the objects to pick and place, and the angular coefficient and intercept of the straight line as goals. It waits for result and then it prints the task status, the detection outcome and the navigation status. In the meantime it receives feedbacks of the task status.

- `apriltags_detection_node`: it has the role of the node B. This node serves to initialize and execute the `AprilTagsDetectionAction` server, that allows to detect in real-time the poses of the AprilTags and the colors of the prisms through the camera.

- `robot_navigation_node`: this node serves to initialize and execute the `RobotNavigationAction` server, that implements the navigation subroutines through the `robot_controller_node`.

- `robot_controller_node`: this node serves to initialize and execute the `RobotControllerAction` server, which sends basic motion commands to the robot, used while controlling the navigation and pick-and-place routines.

- `pick_place_node`: it has the role of the node C. It exploits `MoveIt!` to perform pick-and-place actions on each prism.

- `task_status_node`: it serves to initialize and execute the `TaskStatusAction` server, that manages AprilTags detection, robot navigation and pick-and-place tasks, using the previous four nodes. It keeps receiving feedbacks of the task status.

## Actions

- `AprilTagsDetectionAction`: action for the detection of the AprilTags during navigation. It has as goal the IDs of the AprilTags to be found, and the feedback is composed by the IDs, poses and colors of the AprilTags that TIAGo finds. The result comprises: AprilTag IDs with their poses and colors relative to the `map` frame, and the final status of the detection (succeeded or failed).

- `RobotNavigationAction`: action for TIAGo navigation. It has as goal the path of waypoints, and as feedback the robot state. The result is whether the navigation succeeded or not and the robot final state.

- `RobotControllerAction`: action for TIAGo basic motion control. It has as goal the motion command to perform, and if necessary additional parameters for the movement of robot and its joints. The result is the status of the robot and the reached pose. It has no feedback.

- `PickPlaceAction`: action for TIAGo pick-and-place. It has as goal the motion to perform and additional parameters relative to collision objects, place position and AprilTags information. It has as result the status and collision objects IDs of the tables and the prisms.

- `TaskStatusAction`: action that handles the task pipeline through the AprilTags detection, robot navigation and pick-and-place routines. It has goals, feedbacks and results from the actions described above.

## Configuration files

- `waypoints.yaml`: this file contains the poses of the waypoints with respect to the `map` frame.

- `tables.yaml`: this file contains the geometric information of the prism solids.

- `apriltags.yaml`: this file contains the geometric information of the table solids.

## Launch files

- `env.launch`: it runs the simulation world.

- `task.launch`: it runs the nodes `start_task_node`, `apriltags_detection_node`, `robot_navigation_node`, `robot_controller_node`, `pick_place_node`, `task_status_node`.

- `rviz_pp.launch`: it runs a preconfigured RViz setup by loading the environment with the relative map and reference frames useful for debugging the robot behaviour while performing the task.

# Assumptions
We made the following assumptions:

- the robot reaches the table area using waypoints with `move_base`.

- the pick-and-place routine starts from the left side of the tables.

- the two docking positions along each table side have equal x-coordinate w.r.t. the `map` frame.

- the two docking positions relative to the same table (pick table or place table) have the same y-coordinate w.r.t. the `map` frame.