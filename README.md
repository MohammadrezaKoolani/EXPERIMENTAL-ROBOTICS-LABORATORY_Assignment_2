# EXPERIMENTAL-ROBOTICS-LABORATORY_Assignment_2

# Overview
This ROS package is designed to empower a mobile robot with the capability to autonomously navigate an environment, identify specific markers, and return to its initial location. Utilizing advanced planning with PDDL through ROSPlan, the robot executes a series of actions to achieve its goals efficiently and accurately.
# ObjectivesObjectives
The primary objectives of this project include:
•	Autonomous Navigation: Employing sensor data and mapping techniques to navigate without human intervention.
•	Marker Identification: Detecting and recognizing markers with distinct IDs using onboard camera systems.
•	Action Planning: Integrating ROSPlan to dynamically create and execute plans based on the current state of the environment.
•	Return to Start: Ensuring the robot can return to its starting point after completing its tasks.
# Application Scenario
The robot operates in a predefined area where it must locate and identify markers placed at various locations. This scenario tests the robot's ability to combine perception, planning, and movement to perform tasks that would be valuable in real-world applications such as search and rescue, surveillance, and autonomous delivery systems.


# Running the Simulation
To initiate the simulation, you will need to install the necessary dependencies for planning and navigation.
Begin by navigating to the src folder within your ROS workspace. Install the dependencies for aruco_ros and rosbot_ros with the following commands:


`git clone https://github.com/CarmineD8/aruco_ros.git git clone` `https://github.com/husarion/rosbot_ros.git -b noetic `


**Important: In the launch file** rosbot_ros/src/rosbot_bringup/launch/rosbot_gazebo.launch, remember to insert -y 1.0 into the arguments at line 9.

## aruco_ros
The aruco_ros package is a ROS wrapper for the Aruco Augmented Reality marker detector library, which offers:
•	Efficient AR marker tracking suitable for high-framerate requirements.
•	Creation of AR markers designed to reduce perceptual ambiguity, especially useful when tracking multiple markers.
•	Improved precision with boards of markers for enhanced tracking performance.
•	ROS wrappers to integrate seamlessly with your ROS environment.

IMAGE


## rosbot_ros
The rosbot_ros repository contains ROS2 packages tailored for ROSbot 2R and ROSbot 2 PRO:
•	rosbot: A metapackage including dependencies to other necessary repositories.
•	rosbot_bringup: Contains launch files that initiate all fundamental functionalities and configurations for localization and control.
•	rosbot_description: Provides the URDF model, offering a reliable source for robot transforms and designed to integrate with ROS Industrial standards, pre-configured for ROS2 control.
•	rosbot_gazebo: Includes launch files for the Ignition Gazebo that work with ROS2 control.
•	rosbot_controller: Configuration for ROS2 hardware controllers specific to ROSbots.


IMAGE

## Setting up ROSPlan
To integrate ROSPlan into your system, open a terminal and execute the following commands:

`sudo apt install flex bison freeglut3-dev libbdd-dev python3-catkin-tools ros-noetic-tf2-bullet git clone https://github.com/KCL-Planning/ROSPlan `

Please note: Modify the CMakeLists.txt file within the rosplan_dependencies package by appending -Wno-error=deprecated-copy at line 92 to avoid specific compilation errors related to deprecated features.

### ROSPlan Framework
The ROSPlan framework provides a comprehensive solution for integrating task planning and execution into ROS systems. It simplifies the interface between planning and dispatch and includes connections to prevalent ROS libraries, enhancing the planning capabilities within a ROS environment.
ROSPlan is designed to be versatile, supporting various planning scenarios. It assists in the entire planning lifecycle: from problem domain representation and plan generation to execution and monitoring of actions, making it a robust choice for projects that require automated planning.

### ROSPlan Demos:
Several demos are available in the rosplan_demos repository. More will be added over time.

IMAGE





## Installing the Navigation Packages
For the robot to effectively navigate, you must install the ROS Navigation stack. Within the src folder of your ROS workspace, use the following command:

`git clone https://github.com/ros-planning/navigation.git`

### About the ROS Navigation Stack
The ROS Navigation Stack offers a comprehensive 2D navigation solution, incorporating odometry and sensor data to safely guide a mobile robot to a target position. It processes the incoming information to generate velocity commands that navigate the robot while avoiding obstacles, ensuring smooth and safe operation of the mobile base in diverse environments. The Navigation Stack is fairly simple on a conceptual level. It takes in information from odometry and sensor streams and outputs velocity commands to send to a mobile base. Use of the Navigation Stack on an arbitrary robot, however, is a bit more complicated. As a pre-requisite for navigation stack use, the robot must be running ROS, have a tf transform tree in place, and publish sensor data using the correct ROS Message types. Also, the Navigation Stack needs to be configured for the shape and dynamics of a robot to perform at a high level. To help with this process, this manual is meant to serve as a guide to typical Navigation Stack set-up and configuration. 

IMAGE

## Installing Gmapping
To equip your robot with SLAM capabilities, proceed to install gmapping with these commands:
`sudo apt-get install ros-noetic-openslam-gmapping git clone -b noetic https://github.com/CarmineD8/SLAM_packages.git `

### Understanding Gmapping
Gmapping provides your robot with Simultaneous Localization and Mapping (SLAM) abilities, allowing it to construct a map of an unknown environment while simultaneously keeping track of its location within it. This functionality is crucial for autonomous navigation where the environment is not predefined.
Once you have installed the necessary packages, compile your workspace with catkin_make from the root of your catkin_ws folder.

**Note**: Should you encounter any issues during compilation, particularly with the amcl package conflicting with gmapping, you can resolve this by removing the amcl package from your navigation stack and rerunning the catkin_make command. 

IMAGE

To integrate the project into your ROS environment, clone the repository into the src folder of your workspace with the following command:

`git clone https://github.com/MohammadrezaKoolani/EXPERIMENTAL-ROBOTICS-LABORATORY_Assignment_2.git`

After downloading the project, navigate to your workspace's root folder and build the specific package by running:

`catkin_make --only-pkg-with-deps lab_assignment_2 
`
This will compile only the lab_assignment_2 package and its dependencies, ensuring a targeted build process.

## Launching the Project
Initiate the entire project with the launch file provided:

`roslaunch lab_assignment_2 rosbot.launch `

The rosbot.launch file is configured to set up all necessary nodes, topics, and parameters to start the robot's autonomous mission. This includes initializing the robot's position, setting up marker detection, and activating the navigation stack, allowing the robot to begin its search for the predefined markers within the simulation environment.
By running this launch file, you're bringing the mobile robot to life, enabling it to navigate the virtual space, recognize markers, and perform tasks as dictated by the action plan. This holistic approach simulates a real-world application where the robot's autonomy is crucial.


# Code Explanation and Flowchart

## DecisionMakerNode.py Overview
This script functions as the central decision-making unit, guiding the robot through navigation and marker detection. It dynamically adjusts the robot's actions based on environmental feedback and predefined objectives.
### Key Components Explained
•	**Initialization and ROS Setup**: The node is initialized, and subscribers and publishers are set up to communicate with other parts of the system.
```python
rospy.init_node('logic_node', anonymous=True) self.marker_sub = rospy.Subscriber('/rosbot/marker_found', Int32, self.marker_callback) self.cmd_pub = rospy.Publisher('/rosbot/command', String, queue_size=10) 
```
•	**Marker Detection Callback:** Reacts to markers detected by the robot, triggering navigation if the target marker is found.
```python
if msg.data == self.target_marker_id and self.current_state == "searching": rospy.loginfo("Target marker found. Initiating navigation to marker...") 
```
•	**Navigation to Marker**: Sends a goal to the move_base action server to navigate towards the detected marker.
```python
goal.target_pose.pose = PoseStamped() # Set your goal pose here self.waypoint_client.send_goal(goal) 
```
•	**Post-Navigation Logic:** Once navigation is complete, this function can be customized to perform additional tasks, such as updating the robot's state or preparing for the next goal.
```python
self.cmd_pub.publish("Post-navigation action executed.") 
```
This section highlights the script's ability to integrate perception, decision-making, and action execution, demonstrating how the robot autonomously navigates and responds to its environment.
## VisionProcessingNode.py Overview
This script is responsible for processing visual input from the robot's camera to detect and identify ArUco markers. It employs OpenCV and the ArUco library to recognize markers in the camera feed, determining their IDs and positions.
### Key Components Explained
•	**Setup for Marker Detection**: The script initializes a subscriber to the camera feed and a publisher for detected markers. It uses the ArUco library to define the dictionary and detection parameters for marker identification.
```python
self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250) self.parameters = aruco.DetectorParameters_create() 
```
•	**Marker Detection in Images**: Upon receiving a frame from the camera, the script uses aruco.detectMarkers to find markers in the image. Detected markers are then published, including their ID and center coordinates.
```python
corners, ids, _ = aruco.detectMarkers(cv_image, self.dictionary, parameters=self.parameters) 
```
•	**Publishing Detected Markers:** For each detected marker matching the target ID, the script creates a Marker message with the marker's ID and center position, then publishes it.
```python
marker_msg.id = id[0] marker_msg.center.x = centerX marker_msg.center.y = centerY 
```
This script enhances the robot's ability to interact with its environment by providing it with the capability to recognize specific markers, a crucial step for navigation and task execution.

## find_marker_action_server.py Overview
This script implements the action server part of the ROS action protocol, specifically designed to handle requests to locate specific markers in the robot's environment. It integrates closely with the robot's motion control and vision processing capabilities to achieve this goal.
### Key Components Explained
•	**Action Server Initialization:** The server is set up to respond to FindMarkerAction requests, executing the execute_cb callback when a goal is received.

```python
self._as = actionlib.SimpleActionServer('find_marker', FindMarkerAction, execute_cb=self.execute_cb, auto_start=False) 
```
•	**Goal Processing**: Upon receiving a goal, which includes the ID of the marker to find, the robot begins rotating to scan its surroundings for the marker.
```python
self.rotate_rosbot(0.8) 
```
•	**Marker Detection Callback**: This function is called when a marker is detected. If the detected marker matches the goal ID, it stops the robot and marks the goal as succeeded.
```python
if msg.id == self.id.data: self.marker_found = True 
```
This script is essential for enabling the robot to dynamically seek out and identify markers based on received goals, showcasing the integration of action-based communication, robotic movement, and sensor processing within a ROS framework.

## find_marker_action_client.py Overview
This Python script functions as an action client in the ROS environment, tasked with sending goals to the find_marker_action server. The goals specify the ID of the marker that needs to be found, demonstrating an essential aspect of robot autonomy in dynamic environments.
### Key Components Explained
•	**Initialization and Action Client Setup:** The script initializes a ROS node and creates an action client connected to the find_marker_action server, waiting for the server to become available before sending goals.
```python
self.client = actionlib.SimpleActionClient('find_marker_action', FindMarkerAction) 
```
•	**Sending Goals:** It sends a goal to the server with the specific marker ID to locate and waits for the result, showcasing the use of action clients to perform specific tasks in ROS.
```python
goal = FindMarkerGoal(marker_id=marker_id) self.client.send_goal(goal) 
```
•	Result Handling: After sending the goal, it waits for the action server to complete the task and logs the outcome, illustrating how clients handle responses from action servers.
```python
result = self.client.get_result() 
```
This script exemplifies how to programmatically interact with action servers in ROS, focusing on goal-oriented tasks that require feedback from the environment or other components within a robotic system.

## go_to_waypoint_action.py Overview
This Python script acts as an action server for waypoint navigation, utilizing the move_base action to direct the robot to designated locations. It's integral for tasks requiring precise movement and obstacle avoidance.
### Key Components Explained
•	**Action Server Initialization**: Establishes the server to accept goals for waypoint navigation, illustrating the use of action servers for task-specific movements.
```python
self.server = actionlib.SimpleActionServer('go_to_waypoint', MoveBaseAction, self.execute, False) 
```
•	**Goal Execution: ** The execute callback function processes navigation goals, showcasing how to set up and send a move base goal for navigating to waypoints.
```python
move_base_goal.target_pose.pose = Pose(Point(goal.x, goal.y, 0), Quaternion(0, 0, 0, 1)) 
```
This script highlights the robot's capability to navigate autonomously to specified waypoints, showcasing the integration of ROS action servers with the navigation stack for effective path planning and execution.


# Pseudocode for Autonomous Robot Navigation and Marker Identification

# Initialize ROSPlan with PDDL files
initialize_rosplan_with_pddl()

# Generate problem
problem_generated = generate_problem_request()
if not problem_generated:
    raise Exception("Error in problem generation")

# Generate plan
plan_generated = generate_plan_request()
if not plan_generated:
    raise Exception("Error in plan generation")

# Parse plan
plan_parsed = parse_plan_request()
if not plan_parsed:
    raise Exception("Error in parsing plan")

# Dispatch plan
goal_achieved = dispatch_plan_request()

# Start actions if goal is achieved
if goal_achieved:
    start_actions()

    while True:  # This loop continues until all markers are found or an error occurs
        # Set the next waypoint as the target
        target = set_next_waypoint()

        # Send the goal to MoveBase
        send_goal_to_movebase(target)

        # Drive the robot towards the waypoint
        drive_robot_to_waypoint()

        # If the robot doesn't find the marker, correct its trajectory
        if not find_the_marker():
            correct_trajectory_with_gmapping()

        # If the marker is found, set it as found
        if find_the_marker():
            set_marker_as_found()

            # Check if there are more markers to find
            if not more_markers_to_find():
                # If no more markers, go back to the start
                go_back_to_start()
                break  # End the loop if all markers are found

# If any step fails, an error should be raised
else:
    raise Exception("Failed to achieve the goal")

# End of the operation
end()


