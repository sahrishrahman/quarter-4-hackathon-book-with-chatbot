# Lesson 2: Cognitive Planning - Using LLMs to Translate Natural Language into ROS 2 Actions

## Introduction

Cognitive planning represents the bridge between high-level natural language commands and low-level robotic actions. This lesson explores how Large Language Models (LLMs) can be used to interpret complex natural language instructions and decompose them into sequences of specific ROS 2 actions. You will learn to implement planning systems that can take abstract commands like "Clean the room" and translate them into concrete steps such as "navigate to location A", "detect object B", "grasp object B", and "dispose of object B". This cognitive layer is essential for creating truly autonomous humanoid robots that can understand and execute complex tasks.

## Hands-on Exercise

Implement a cognitive planning system using LLMs:
1.  **Set up LLM Interface**: Configure access to an LLM API (e.g., OpenAI GPT, Anthropic Claude) for processing natural language commands.
2.  **Define Action Vocabulary**: Create a structured vocabulary of available robot actions and their parameters that the LLM can use in its plans.
3.  **Implement Planning Pipeline**: Build a system that receives natural language commands, processes them through the LLM, and generates executable action sequences.
4.  **Action Validation**: Implement validation mechanisms to ensure the generated action sequences are feasible and safe for execution.
5.  **Test Complex Commands**: Test the system with complex, multi-step commands to evaluate the LLM's ability to generate appropriate action sequences.

Sample cognitive planning implementation:
```python
import rospy
import openai
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import json

class CognitivePlanner:
    def __init__(self):
        rospy.init_node('cognitive_planner')

        # Set up OpenAI API (replace with your API key)
        openai.api_key = "YOUR_API_KEY_HERE"

        # Publishers for different robot capabilities
        self.nav_pub = rospy.Publisher('/move_base/goal', Pose, queue_size=10)
        self.cmd_pub = rospy.Publisher('/robot_command', String, queue_size=10)

        # Define available actions and their parameters
        self.action_definitions = """
        Available Actions:
        1. NAVIGATE_TO(location): Move robot to specified location
        2. DETECT_OBJECT(object_type): Use vision system to find specific objects
        3. GRASP_OBJECT(object_id): Pick up identified object
        4. RELEASE_OBJECT(): Put down currently held object
        5. SCAN_AREA(): Perform 360-degree environment scan
        6. RETURN_HOME(): Navigate back to charging station
        """

    def plan_from_command(self, natural_language_command):
        """
        Use LLM to generate action plan from natural language command
        """
        prompt = f"""
        You are a cognitive planning system for a humanoid robot. Your task is to convert natural language commands into sequences of specific robot actions.

        {self.action_definitions}

        The robot's current state is: at home position, not holding any object, sensors operational.

        Natural language command: "{natural_language_command}"

        Respond with a JSON array of actions in the following format:
        [
            {{"action": "NAVIGATE_TO", "parameters": {{"location": "living_room"}}},
            {{"action": "DETECT_OBJECT", "parameters": {{"object_type": "trash"}}},
            {{"action": "GRASP_OBJECT", "parameters": {{"object_id": "can_001"}}}
        ]

        Be specific about locations and objects. Only include actions that are relevant to the command.
        """

        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}],
                temperature=0.1
            )

            # Extract the JSON from the response
            content = response.choices[0].message.content
            # In practice, you'd want to parse this more robustly
            start = content.find('[')
            end = content.rfind(']') + 1
            if start != -1 and end != 0:
                action_json = content[start:end]
                plan = json.loads(action_json)
                return plan
            else:
                raise ValueError("Could not extract action plan from response")

        except Exception as e:
            rospy.logerr(f"Error generating plan: {e}")
            return []

    def execute_plan(self, plan):
        """
        Execute the planned sequence of actions
        """
        for action in plan:
            action_name = action['action']
            parameters = action.get('parameters', {})

            rospy.loginfo(f"Executing: {action_name} with {parameters}")

            if action_name == "NAVIGATE_TO":
                self.navigate_to(parameters['location'])
            elif action_name == "DETECT_OBJECT":
                self.detect_object(parameters['object_type'])
            elif action_name == "GRASP_OBJECT":
                self.grasp_object(parameters['object_id'])
            elif action_name == "RELEASE_OBJECT":
                self.release_object()
            elif action_name == "SCAN_AREA":
                self.scan_area()
            elif action_name == "RETURN_HOME":
                self.return_home()

            rospy.sleep(1.0)  # Wait between actions

    def navigate_to(self, location):
        # Implementation would map location names to coordinates
        rospy.loginfo(f"Navigating to {location}")
        # Publish navigation goal

    def detect_object(self, object_type):
        rospy.loginfo(f"Detecting {object_type}")
        # Trigger object detection system

    def grasp_object(self, object_id):
        rospy.loginfo(f"Grasping {object_id}")
        # Trigger grasping action

    def release_object(self):
        rospy.loginfo("Releasing object")
        # Trigger release action

    def scan_area(self):
        rospy.loginfo("Scanning area")
        # Trigger 360-degree scan

    def return_home(self):
        rospy.loginfo("Returning home")
        # Navigate to home position

# Example usage
def main():
    planner = CognitivePlanner()

    # Example command
    command = "Clean the living room by picking up any trash you find"
    plan = planner.plan_from_command(command)

    rospy.loginfo(f"Generated plan: {plan}")

    # Execute the plan (in a real system, you'd validate first)
    # planner.execute_plan(plan)

if __name__ == '__main__':
    main()
```

## Summary

Cognitive planning using LLMs enables robots to understand and execute complex natural language commands by translating them into sequences of specific actions. This cognitive layer is crucial for creating autonomous humanoid robots that can operate in human environments with minimal direct programming. The combination of LLMs with ROS 2 provides a powerful framework for high-level robot control and task execution.