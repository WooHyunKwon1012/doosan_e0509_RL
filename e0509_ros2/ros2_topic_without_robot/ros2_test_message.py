import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp({"renderer": "RayTracedLighting", "headless": True})

import omni
from isaacsim.core.utils.extensions import enable_extension

# enable ROS2 bridge extension
enable_extension("isaacsim.ros2.bridge")

# Make the rclpy imports
import rclpy
from custom_message.msg import SampleMsg

# Create message
sample_msg = SampleMsg()

# assign data in the string part and integer part of the message
sample_msg.my_string.data = "hello from Isaac Sim!"
sample_msg.my_num = 23

print("Message assignment completed!")
