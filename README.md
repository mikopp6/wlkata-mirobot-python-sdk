

# wlkata-mirobot-python-sdk

Translated to English with AI

Wlkata Intelligent Mirobot 6-DOF Robotic Arm Python SDK



## Preparation

* Power on the robotic arm and connect it to the computer's USB port.

* Ensure that the computer has the CH340 driver installed.

* Install the Mirobot Python SDK.

## Quick Start

```python
'''
Control the wrist joint of the robotic arm, point-to-point (P2P) mode
'''
import time
from wlkata_mirobot import WlkataMirobot

# Create the robotic arm
arm = WlkataMirobot()
# Return the robotic arm to the home position (synchronous mode)
arm.home()

print("Moving to target point A")
arm.set_tool_pose(200,  20, 230)
print(f"Current end effector pose in the robotic arm coordinate system: {arm.pose}")
time.sleep(1)

print("Moving to target point B")
arm.set_tool_pose(200,  20, 150)
print(f"Current end effector pose in the robotic arm coordinate system: {arm.pose}")
time.sleep(1)

print("Moving to target point C, specifying the end effector's orientation angles")
arm.set_tool_pose(150,  -20,  230, roll=30.0, pitch=0, yaw=45.0)
print(f"Current end effector pose in the robotic arm coordinate system: {arm.pose}")
```

## User Manual

For detailed API documentation and example code, please refer to `doc/WLKATA MIROBOT Python SDK User Manual/`
