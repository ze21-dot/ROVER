# İTÜ ROVER

## 1- Create a Workspace
```
#Creating new directory called "rover_ws" in the home folder
mkdir ~/rover_ws

cd ~/rover_ws
catkin_init_workspace

# this is where packages will go
mkdir src
```

## 2- Clone the Given Package
```
cd ~/rover_ws/src

# Clone the given itü github  
git clone https://github.com/itu-rover/iturover_keyboard_control_assignment.git

# Return to the workspace root directory.
cd ..

# compile the newly cloned package.
catkin build
```

## 3- Create a New ROS Package-
```
cd ~/rover_ws/src

# Creating ROS package named "reactive_robot" with the required dependencies
catkin_create_pkg reactive_robot std_msgs rospy geometry_msgs message_generation

cd reactive_robot

# Create a folder to contain launch files
mkdir launch
```

## 4- Python Script
```
cd ~/rover_ws/src/reactive_robot

# Creating  folder to store Python scripts
mkdir scripts

# Python file named "reactive.py."
touch scripts/reactive.py

# Make py. script executable
chmod +x scripts/reactive.py
```

## 5- Python Script (reactive.py)
```
#!/usr/bin/env python3

# Import necessary libraries
import rospy  # ROS Python library
from geometry_msgs.msg import Twist  # Message type for velocity commands
from pynput import keyboard  # Library to get keyboard input
import threading


# Defining given variables and constants
TIME_LIMIT = 60  # Maximum time (seconds) for robot motion
motion_active = False  # Flag to track if the robot is moving
velocity_pub = None  # ROS publisher for velocity commands
current_command = None  # Currently pressed keyboard key

def on_press(key):
    """Handle key press events."""
    global current_command, motion_active
    try:
        # check the current command based on w, a, s, or d
        if key.char in ['w', 'a', 's', 'd']:
            current_command = key.char
            motion_active = True
            rospy.loginfo(f"Key pressed: {key.char}")
    except AttributeError:
        pass

def on_release(key):
    """Handle key release events (stop the robot)."""
    global current_command
    current_command = None

def keyboard_listener():
    """Listen for keyboard input in a separate thread."""
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

def motion_control():
    """Control the robot's motion based on keyboard input."""
    global motion_active, current_command

    # Define a rate of 10 Hz for sending velocity commands
    rate = rospy.Rate(10)
    vel_msg = Twist()  # Message to store velocity commands

    # Track the start time to enforce the time limit
    start_time = rospy.get_time()
    while rospy.get_time() - start_time < TIME_LIMIT:
        # get movement commands from pressed key.
        if current_command == 'w':  # Move forward
            vel_msg.linear.x = 0.5
            vel_msg.angular.z = 0.0
            rospy.loginfo("Moving forward.")
        elif current_command == 's':  # Move backward
            vel_msg.linear.x = -0.5
            vel_msg.angular.z = 0.0
            rospy.loginfo("Moving backward.")
        elif current_command == 'a':  # Turn left
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.5
            rospy.loginfo("Turning left.")
        elif current_command == 'd':  # Turn right
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = -0.5
            rospy.loginfo("Turning right.")
        else:  # Stop if no key is pressed
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0

        # Publish the velocity command to the robot.
        velocity_pub.publish(vel_msg)
        rate.sleep()

    # Stop the robot when the time limit is reached.
    rospy.loginfo("Time limit reached. Stopping...")
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = 0.0
    velocity_pub.publish(vel_msg)

def main():
    """Main function to initialize the ROS node and start the robot control."""
    global velocity_pub

    # start the ROS node.
    rospy.init_node('reactive_robot_teleop', anonymous=True)

    # Create a publisher to send velocity commands to the robot.
    velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rospy.loginfo("Node started. Listening for keyboard inputs...")

    # Start the keyboard listener 
    thread = threading.Thread(target=keyboard_listener, daemon=True)
    thread.start()

    # Start controlling the robot.
    motion_control()

    rospy.loginfo("Shutting down node.")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

```

## 6- Create a Launch File

```
# Creating launch file in the "launch" folder.
touch launch/reactive_robot.launch
```
```
<launch>
    <!-- cloned esimulation -->
    <include file="$(find iturover_keyboard_control_assignment)/launch/reactive_robot_sim.launch" />

    <!-- Py script-->
    <node pkg="reactive_robot" type="reactive.py" name="reactive_robot_teleop" output="screen" />
</launch>

```

## 7- Install Required Packages
```
# Install the TurtleBot3 Gazebo simulation package
sudo apt install ros-noetic-turtlebot3-simulations

```

## 8- Build and Source the Workspace
```
cd ~/rover_ws
catkin build
source devel/setup.bash
```

## 9-Run the Launch File
```
roslaunch reactive_robot_sim reactive_robot_sim.launch
```

![the end](https://github.com/ze21-dot/ROVER/blob/main/rover.jpg.png?raw=true)



















