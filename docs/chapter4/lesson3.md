# Lesson 3: Capstone Project - The Autonomous Humanoid

## Introduction

The capstone project integrates all the concepts learned throughout the course to create a fully autonomous humanoid robot system. This project combines ROS 2 communication, simulation environments, NVIDIA Isaac technologies, voice recognition, and cognitive planning into a comprehensive system. You will implement a complete pipeline where a simulated humanoid robot receives a voice command, plans a path, navigates obstacles, identifies an object using computer vision, and manipulates it. This project demonstrates the convergence of all the technologies covered in the previous modules.

## Hands-on Exercise

Build and test the complete autonomous humanoid system:
1.  **System Integration**: Integrate all components (ROS 2 nodes, Isaac Sim, Whisper, LLM planning) into a unified system architecture.
2.  **Voice Command Processing**: Implement the complete pipeline from voice input through Whisper to LLM cognitive planning.
3.  **Navigation and Path Planning**: Set up Nav2 with Isaac ROS for VSLAM-based navigation in the simulated environment.
4.  **Object Detection and Manipulation**: Configure computer vision systems for object detection and robotic manipulation planning.
5.  **Complete Task Execution**: Execute a full task such as "Robot, please go to the kitchen, find the red cup, and move it to the table" and observe the complete autonomous execution.

Sample integrated system architecture:
```python
#!/usr/bin/env python3

import rospy
import whisper
import openai
import json
import pyaudio
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2

class AutonomousHumanoid:
    def __init__(self):
        rospy.init_node('autonomous_humanoid')

        # Initialize components
        self.whisper_model = whisper.load_model("base")
        self.cv_bridge = CvBridge()

        # Audio parameters for voice recognition
        self.chunk = 1024
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 44100
        self.audio = pyaudio.PyAudio()

        # Publishers and subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.nav_goal_pub = rospy.Publisher('/move_base_simple/goal', Pose, queue_size=10)
        self.camera_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.camera_callback)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)

        # System state
        self.current_task = None
        self.detected_objects = []
        self.is_moving = False

        # Command mapping for direct control
        self.command_map = {
            "move forward": self.move_forward,
            "move backward": self.move_backward,
            "turn left": self.turn_left,
            "turn right": self.turn_right,
        }

    def camera_callback(self, data):
        """Process camera data for object detection"""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(data, "bgr8")
            # In a real system, you'd run object detection here
            # For this example, we'll simulate object detection
            self.process_image(cv_image)
        except Exception as e:
            rospy.logerr(f"Error processing camera data: {e}")

    def laser_callback(self, data):
        """Process laser scan data for navigation"""
        # Process laser data for obstacle detection
        pass

    def process_image(self, image):
        """Simple object detection (in practice, use a trained model)"""
        # Convert to HSV for color-based detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define range for red color (simplified)
        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 50, 50])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2

        # Find contours of detected objects
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            if cv2.contourArea(contour) > 1000:  # Filter small objects
                # Calculate object position
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    self.detected_objects.append((cx, cy, "red_object"))

    def voice_command_handler(self):
        """Handle voice commands using Whisper and LLM planning"""
        # This would run in a separate thread in practice
        rospy.loginfo("Listening for voice commands...")

        # In practice, you'd have a continuous listening loop
        # For this example, we'll simulate a command
        command = "Please go to the kitchen, find the red cup, and bring it to the table"

        # Process through LLM to generate action plan
        plan = self.generate_action_plan(command)

        if plan:
            rospy.loginfo(f"Generated plan: {plan}")
            self.execute_action_plan(plan)

    def generate_action_plan(self, command):
        """Generate action plan using LLM"""
        prompt = f"""
        You are a cognitive planning system for an autonomous humanoid robot. Convert the following command into a sequence of specific actions.

        Available Actions:
        - NAVIGATE_TO(location)
        - DETECT_OBJECT(type)
        - GRASP_OBJECT(id)
        - RELEASE_OBJECT()
        - SCAN_AREA()
        - REPORT_STATUS()

        Current robot state: In living room, no object in hand, sensors operational.

        Command: "{command}"

        Respond with a JSON array of actions:
        [
            {{"action": "NAVIGATE_TO", "parameters": {{"location": "kitchen"}}},
            {{"action": "SCAN_AREA", "parameters": {{}}}
        ]
        """

        try:
            # In practice, use actual LLM API
            # For this example, we'll return a predefined plan
            return [
                {"action": "NAVIGATE_TO", "parameters": {"location": "kitchen"}},
                {"action": "SCAN_AREA", "parameters": {}},
                {"action": "DETECT_OBJECT", "parameters": {"type": "red_cup"}},
                {"action": "GRASP_OBJECT", "parameters": {"id": "red_cup_001"}},
                {"action": "NAVIGATE_TO", "parameters": {"location": "table"}},
                {"action": "RELEASE_OBJECT", "parameters": {}}
            ]
        except Exception as e:
            rospy.logerr(f"Error generating plan: {e}")
            return []

    def execute_action_plan(self, plan):
        """Execute the generated action plan"""
        for action in plan:
            action_name = action['action']
            params = action.get('parameters', {})

            rospy.loginfo(f"Executing action: {action_name} with {params}")

            if action_name == "NAVIGATE_TO":
                self.navigate_to_location(params['location'])
            elif action_name == "DETECT_OBJECT":
                self.detect_specific_object(params['type'])
            elif action_name == "GRASP_OBJECT":
                self.grasp_object(params['id'])
            elif action_name == "RELEASE_OBJECT":
                self.release_object()
            elif action_name == "SCAN_AREA":
                self.scan_environment()

            rospy.sleep(2.0)  # Wait between actions

    def navigate_to_location(self, location):
        """Navigate to specified location"""
        rospy.loginfo(f"Navigating to {location}")
        # In practice, this would send navigation goals to Nav2
        # For simulation, we'll just move forward briefly
        self.move_forward()

    def detect_specific_object(self, obj_type):
        """Detect specific object type"""
        rospy.loginfo(f"Looking for {obj_type}")
        # The camera callback would handle detection
        rospy.sleep(3.0)  # Simulate search time

    def grasp_object(self, obj_id):
        """Grasp the specified object"""
        rospy.loginfo(f"Grasping {obj_id}")
        # In practice, this would trigger the manipulator

    def release_object(self):
        """Release currently held object"""
        rospy.loginfo("Releasing object")
        # In practice, this would trigger the manipulator

    def scan_environment(self):
        """Scan the environment"""
        rospy.loginfo("Scanning environment")
        # In practice, this would process sensor data

    def move_forward(self):
        """Move robot forward"""
        msg = Twist()
        msg.linear.x = 0.3
        self.cmd_vel_pub.publish(msg)
        rospy.sleep(1.0)
        self.stop_robot()

    def stop_robot(self):
        """Stop robot movement"""
        msg = Twist()
        self.cmd_vel_pub.publish(msg)

    def run(self):
        """Main execution loop"""
        rospy.loginfo("Autonomous Humanoid System Starting...")

        # In a real system, you'd have continuous operation
        # For this example, we'll execute one cycle
        self.voice_command_handler()

        rospy.loginfo("Task completed. System shutting down.")
        rospy.spin()

if __name__ == '__main__':
    try:
        robot = AutonomousHumanoid()
        robot.run()
    except rospy.ROSInterruptException:
        pass
```

## Summary

The capstone project demonstrates the integration of all technologies covered in the course into a complete autonomous humanoid system. By combining voice recognition, cognitive planning, navigation, perception, and manipulation, students create a robot that can understand natural language commands and execute complex tasks autonomously. This project showcases the convergence of multiple advanced technologies that define the future of physical AI and humanoid robotics.