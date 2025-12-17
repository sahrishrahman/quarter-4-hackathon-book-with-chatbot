# Lesson 1: Voice-to-Action - Using OpenAI Whisper for Voice Commands

## Introduction

The integration of voice recognition with robotic action represents a significant advancement in human-robot interaction. This lesson explores how OpenAI Whisper can be used to convert human voice commands into actionable instructions for robots. You will learn to implement voice recognition systems that can understand natural language commands and translate them into robot actions, enabling more intuitive and natural interaction with humanoid robots. This technology is fundamental to creating robots that can respond to human speech in real-world environments.

## Hands-on Exercise

Implement a voice-to-action system using OpenAI Whisper:
1.  **Set up Whisper**: Install OpenAI Whisper and verify that your system can process audio input for speech recognition.
2.  **Create Audio Pipeline**: Build a real-time audio input pipeline that captures voice commands and processes them through Whisper for transcription.
3.  **Command Recognition**: Implement a command recognition system that identifies specific robot commands from the transcribed text (e.g., "move forward", "pick up the object", "turn left").
4.  **Action Mapping**: Create a mapping system that translates recognized commands into specific ROS 2 actions or robot behaviors.
5.  **Test Voice Control**: Test the system with various voice commands to control a simulated robot, observing the accuracy and responsiveness of the voice-to-action pipeline.

Sample Whisper integration code:
```python
import rospy
import whisper
import pyaudio
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class VoiceToAction:
    def __init__(self):
        rospy.init_node('voice_to_action')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Load Whisper model
        self.model = whisper.load_model("base")

        # Audio parameters
        self.chunk = 1024
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 44100
        self.audio = pyaudio.PyAudio()

        # Start audio stream
        self.stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )

        # Command mapping
        self.command_map = {
            "move forward": self.move_forward,
            "move backward": self.move_backward,
            "turn left": self.turn_left,
            "turn right": self.turn_right,
            "stop": self.stop_robot
        }

    def transcribe_audio(self, audio_data):
        # Convert audio data to numpy array
        audio_array = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32)
        audio_array /= 32768.0  # Normalize to [-1, 1]

        # Transcribe using Whisper
        result = self.model.transcribe(audio_array)
        return result["text"]

    def execute_command(self, text):
        text = text.lower().strip()
        rospy.loginfo(f"Recognized command: {text}")

        for command, action in self.command_map.items():
            if command in text:
                rospy.loginfo(f"Executing command: {command}")
                action()
                return

        rospy.loginfo("Command not recognized")

    def move_forward(self):
        msg = Twist()
        msg.linear.x = 0.5  # Move forward at 0.5 m/s
        self.pub.publish(msg)

    def move_backward(self):
        msg = Twist()
        msg.linear.x = -0.5  # Move backward at 0.5 m/s
        self.pub.publish(msg)

    def turn_left(self):
        msg = Twist()
        msg.angular.z = 0.5  # Turn left at 0.5 rad/s
        self.pub.publish(msg)

    def turn_right(self):
        msg = Twist()
        msg.angular.z = -0.5  # Turn right at 0.5 rad/s
        self.pub.publish(msg)

    def stop_robot(self):
        msg = Twist()
        self.pub.publish(msg)  # Stop robot

    def run(self):
        while not rospy.is_shutdown():
            # Read audio data
            data = self.stream.read(self.chunk)

            # Process audio in chunks (in a real system, you'd buffer and process complete phrases)
            # For this example, we'll simulate processing when certain conditions are met
            if np.mean(np.abs(np.frombuffer(data, dtype=np.int16))) > 1000:  # Detect voice activity
                # In practice, you'd buffer several chunks before processing
                rospy.sleep(1.0)  # Wait for complete phrase
                # Transcribe and execute (simplified for example)
                print("Voice detected - would process command in real system")

if __name__ == '__main__':
    try:
        vta = VoiceToAction()
        vta.run()
    except rospy.ROSInterruptException:
        pass
```

## Summary

Voice-to-action systems using OpenAI Whisper enable natural human-robot interaction by converting spoken commands into robotic actions. The combination of advanced speech recognition and robotic control creates intuitive interfaces for humanoid robots. Proper implementation requires careful attention to audio processing, command recognition accuracy, and responsive action execution to create seamless voice-controlled robot systems.