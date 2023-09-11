'''
Mirobot GCode Communication Protocol
'''
import math
from collections.abc import Collection
from contextlib import AbstractContextManager
import logging
import os
from pathlib import Path
import re
import time
from typing import TextIO, BinaryIO
import math
from collections import namedtuple
from enum import Enum
from typing import NamedTuple

try:
	import importlib.resources as pkg_resources
except ImportError:
	# Try backported to PY<37 `importlib_resources`.
	import importlib_resources as pkg_resources

# from .wlkata_mirobot_interface_bluetooth import BluetoothLowEnergyInterface
from .wlkata_mirobot_serial import WlkataMirobotSerial
from .wlkata_mirobot_status import MirobotStatus, MirobotAngles, MirobotCartesians
from .wlkata_mirobot_exceptions import ExitOnExceptionStreamHandler, MirobotError, MirobotAlarm, MirobotReset, MirobotAmbiguousPort, MirobotStatusError, MirobotResetFileError, MirobotVariableCommandError

# Determine the Type of Operating System
# nt: Microsoft NT Standard
os_is_nt = os.name == 'nt'
# Both Linux and Mac belong to the POSIX standard.
# POSIX: A portable API for Unix-like operating systems.
os_is_posix = os.name == 'posix'


class WlkataMirobotTool(Enum):
	NO_TOOL = 0         # No tools
	SUCTION_CUP = 1     # Air pump suction head
	GRIPPER = 2         # Servo gripper
	FLEXIBLE_CLAW = 3   # Three-finger soft gripper
	
class WlkataMirobot(AbstractContextManager):
	'''Wlkata Python SDK'''
	# Air pump PWM value
	AIR_PUMP_OFF_PWM_VALUE = 0
	AIR_PUMP_BLOWING_PWM_VALUE = 500
	AIR_PUMP_SUCTION_PWM_VALUE = 1000
	# Solenoid valve PWM value
	VALVE_OFF_PWM_VALUE = 65
	VALVE_ON_PWM_VALUE = 40 
	# PWM values for the mechanical claw opening and closing
	GRIPPER_OPEN_PWM_VALUE = 40
	GRIPPER_CLOSE_PWM_VALUE = 60
	# Claw spacing range (in mm)
	GRIPPER_SPACING_MIN = 0.0
	GRIPPER_SPACING_MAX = 30.0
	# Claw kinematic parameters definition (in mm)
	GRIPPER_LINK_A = 9.5    # Distance between servo disc and centerline
	GRIPPER_LINK_B = 18.0   # Length of the connecting rod
	GRIPPER_LINK_C = 3.0    # Size of parallel claws when retracted
	
	def __init__(self, *device_args, portname=None, debug=False, connection_type='serial', \
		autoconnect=True, autofindport=True, exclusive=True, \
		default_speed=2000, reset_file=None, wait_ok=False, **device_kwargs):
		'''Initialization'''
  		# Set log level
		self.logger = logging.getLogger(__name__)
		if (debug):
			self.logger.setLevel(logging.DEBUG)
		else:
			self.logger.setLevel(logging.ERROR)

		self.stream_handler = ExitOnExceptionStreamHandler()
		self.stream_handler.setLevel(logging.DEBUG if debug else logging.INFO)

		formatter = logging.Formatter(f"[Mirobot Init] [%(levelname)s] %(message)s")
		self.stream_handler.setFormatter(formatter)
		self.logger.addHandler(self.stream_handler)

		self.device = None
		
		# Create serial connection
		if connection_type.lower() in ('serial', 'ser'):
			# Create serial connection
			serial_device_init_fn = WlkataMirobotSerial.__init__
			args_names = serial_device_init_fn.__code__.co_varnames[:serial_device_init_fn.__code__.co_argcount]
			args_dict = dict(zip(args_names, device_args))
			args_dict.update(device_kwargs)
			
			args_dict['mirobot'] = self
			args_dict['portname'] = portname
			args_dict['exclusive'] = exclusive
			args_dict['debug'] = debug
			args_dict['logger'] = self.logger
			args_dict['autofindport'] = autofindport
			# Set device to serial interface
			self.device = WlkataMirobotSerial(**args_dict)
			# Set port name
			self.default_portname = self.device.default_portname

		formatter = logging.Formatter(f"[{self.default_portname}] [%(levelname)s] %(message)s")
		self.stream_handler.setFormatter(formatter)
		# Read reset file
		self.reset_file = pkg_resources.read_text('wlkata_mirobot.resources', 'reset.xml') if reset_file is None else reset_file
		# Set debug mode
		self._debug = debug
		# Air pump PWM value
		self.pump_pwm_values = [
			self.AIR_PUMP_SUCTION_PWM_VALUE,
			self.AIR_PUMP_BLOWING_PWM_VALUE,
			self.AIR_PUMP_OFF_PWM_VALUE
		]
		# Solenoid valve PWM value
		self.valve_pwm_values = [
			self.VALVE_OFF_PWM_VALUE,
			self.VALVE_ON_PWM_VALUE
		]
		# End effector default movement speed
		self.default_speed = default_speed
		# Whether to wait for 'ok' data to be returned by default
		self.wait_ok = wait_ok
		# Mirobot status information
		self.status = MirobotStatus()
		# Set end effector tool
		self.tool = WlkataMirobotTool.NO_TOOL
		# Auto connect
		if autoconnect:
			self.connect()

	def __enter__(self):
		""" Magic method for contextManagers """
		return self

	def __exit__(self, *exc):
		""" Magic method for contextManagers """
		self.disconnect()

	def __del__(self):
		""" Magic method for object deletion """
		self.disconnect()

	def connect(self):
		"""Connect to the device"""
		self.device.connect()

	def disconnect(self):
		if getattr(self, 'device', None) is not None:
			self.device.disconnect()

	@property
	def is_connected(self):
		return self.device.is_connected

	@property
	def debug(self):
		'''Get the current debug mode (whether it's enabled)'''
		return self._debug

	@debug.setter
	def debug(self, enable):
		'''Set to debug mode'''
		self._debug = bool(enable)
		self.stream_handler.setLevel(logging.DEBUG if self._debug else logging.INFO)
		self.device.setDebug(enable)

	def send_msg(self, msg, var_command=False, disable_debug=False, terminator=os.linesep, wait_ok=None, wait_idle=False):
		'''Send commands to Mirobot'''
		if self.is_connected:
			# Convert to a string from bytes
			# Convert a string to bytes
			if isinstance(msg, bytes):
				msg = str(msg, 'utf-8')
			
			# Remove any newlines
			msg = msg.strip()

			# Check if this is supposed to be a variable command and fail if not
			# If it's a numerical setting command, perform validation
			if var_command and not re.fullmatch(r'\$\d+=[\d\.]+', msg):
				self.logger.exception(MirobotVariableCommandError("Message is not a variable command: " + msg))

			if wait_ok is None:
				wait_ok = False
			
			# print(f"send_msg wait_ok = {wait_ok}")
			# Actually send the message
			# Returns a boolean value representing whether it was sent correctly
			ret = self.device.send(msg,
									  disable_debug=disable_debug,
									  terminator=os.linesep,
									  wait_ok=wait_ok,
									  wait_idle=wait_idle)

			return ret

		else:
			raise Exception('Mirobot is not Connected!')

	def send_cmd_get_status(self, disable_debug=False):
		'''Get Mirobot's status information, returns status characters'''
		instruction = '?'
		ret = self.send_msg(instruction, disable_debug=disable_debug, wait_ok=False, wait_idle=False)
		recv_str = self.device.serial_device.readline(timeout=0.10)
		self.logger.debug(f"[RECV] {recv_str}")
		return recv_str

	def get_status(self, disable_debug=False):
		'''Get and update Mirobot's status'''
		# Because there is a possibility of message loss, multiple queries are needed
		# print("Get Status ...")
		status = None
		while True:	
			msg_seg = self.send_cmd_get_status(disable_debug=disable_debug)
			if "<" in msg_seg and ">" in msg_seg:
				status_msg = msg_seg
				try:
					# print("Start parsing status string")
					ret, status = self._parse_status(status_msg)
					# print(f"Status parsing: {ret}")
					if ret:
						break
				except Exception as e:
					logging.error(e)
			# Wait for a while
			time.sleep(0.1)
		# Set the current status
		self._set_status(status)
		return status
	
	def _set_status(self, status):
		'''Set a new status'''
		self.status = status

	def _parse_status(self, msg):
		'''
		Parse the status information returned by Mirobot from a string, extract relevant variables, and assign them to the robotic arm object
		'''
		return_status = MirobotStatus()
		# Match with regular expressions
		state_regex = r'<([^,]*),Angle\(ABCDXYZ\):([-\.\d,]*),Cartesian coordinate\(XYZ RxRyRz\):([-.\d,]*),Pump PWM:(\d+),Valve PWM:(\d+),Motion_MODE:(\d)>'
		# While re.search() searches for the whole string even if the string contains multi-lines and tries to find a match of the substring in all the lines of the string.
		# Note: Change re.match to re.search
		regex_match = re.search(state_regex, msg)# re.fullmatch(state_regex, msg)
		
		if regex_match:
			try:
				state, angles, cartesians, pump_pwm, valve_pwm, motion_mode = regex_match.groups()

				return_angles = MirobotAngles(**dict(zip('xyzdabc', map(float, angles.split(',')))))

				return_cartesians = MirobotCartesians(*map(float, cartesians.split(',')))

				return_status = MirobotStatus(state,
											  return_angles,
											  return_cartesians,
											  int(pump_pwm),
											  int(valve_pwm),
											  bool(motion_mode))
				self.logger.info(f"state: {state} angle: {return_angles} cartesians: {return_cartesians}")
				self.logger.info(f"pump_pwm: {pump_pwm}, valve_pwm: {valve_pwm}, motion_mode:{motion_mode}")
				return True, return_status
			except Exception as exception:
				self.logger.exception(MirobotStatusError(f"Could not parse status message \"{msg}\" \n{str(exception)}"),
									   exc_info=exception)
		else:
			self.logger.error(MirobotStatusError(f"Could not parse status message \"{msg}\""))
		
		return False, None

	def home(self, has_slider=False):
		'''Robotic Arm Homing'''
		if has_slider:
			return self.home_7axis()
		else:
			return self.home_6axis()
	
	def home_slider(self):
		'''Slider Only Homing'''
		return self.home_1axis(7)
	
	def home_1axis(self, axis_id):
		'''Single Axis Homing'''
		if not isinstance(axis_id, int) or not (axis_id >= 1 and axis_id <= 7):
			return False
		msg = f'$h{axis_id}'
		return self.send_msg(msg, wait_ok=False, wait_idle=True)
	
	def home_6axis(self):
		'''Six-Axis Homing'''
		msg = f'$h'
		return self.send_msg(msg, wait_ok=False, wait_idle=True)
	
	def home_6axis_in_turn(self):
		'''Six-Axis Homing, each joint homing sequentially'''
		msg = f'$hh'
		return self.send_msg(msg, wait_ok=False, wait_idle=True)
	
	def home_7axis(self):
		'''Seven-Axis Homing (Main Body + Slider)'''
		msg = f'$h0'
		return self.send_msg(msg, wait_ok=False, wait_idle=True)
		
	def unlock_all_axis(self):
		'''Unlock the locked state of each axis'''
		msg = 'M50'
		return self.send_msg(msg, wait_ok=True, wait_idle=True)
		
	def go_to_zero(self):
		'''Return to Zero - Move to nominal zero points for each axis'''
		msg = 'M21 G90 G00 X0 Y0 Z0 A0 B0 C0 F2000'
		return self.send_msg(msg, wait_ok=True, wait_idle=True)
	
	def set_speed(self, speed):
		'''Set speed'''
		# Convert to an integer
		speed = int(speed)
		# Check if the value range is legal
		if speed <= 0 or speed > 3000:
			self.logger.error(MirobotStatusError(f"Illegal movement speed {speed}"))
			return False
		# Send a command
		msg = f'F{speed}'
		return self.send_msg(msg, wait_ok=None, wait_idle=None)
	
	def set_hard_limit(self, enable):
		'''
		Enable hardware limits.
		'''
		msg = f'$21={int(enable)}'
		return self.send_msg(msg, var_command=True, wait_ok=None)

	def set_soft_limit(self, enable):
		'''
		Enable software limits
		Note: Please use with caution
		'''
		msg = f'$20={int(enable)}'
		return self.send_msg(msg, var_command=True, wait_ok=None)
	
	def format_float_value(self, value):
		if value is None:
			return value
		if isinstance(value, float):
			# Accurate to two decimal places
			return round(value , 2)
		else:
			return value
	
	def generate_args_string(self, instruction, pairings):
		'''Generate parameter characters'''
		args = [f'{arg_key}{self.format_float_value(value)}' for arg_key, value in pairings.items() if value is not None]

		return ' '.join([instruction] + args)
	
	def set_joint_angle(self, joint_angles, speed=None, is_relative=False, wait_ok=None):
		'''
		Set the joint angles of the robotic arm
		joint_angles: Target joint angle dictionary, where the key is the joint's ID, and the value is the angle (unit: °)
		Example: {1: 45.0, 2: -30.0}
		'''
		for joint_i in range(1, 8):
			# Fill in missing angles
			if joint_i not in joint_angles:
				joint_angles[joint_i] = None

		return self.go_to_axis(x=joint_angles[1], y=joint_angles[2], z=joint_angles[3], a=joint_angles[4], \
			b=joint_angles[5], c=joint_angles[6], d=joint_angles[7], is_relative=is_relative, speed=speed, wait_ok=wait_ok)

	def go_to_axis(self, x=None, y=None, z=None, a=None, b=None, c=None, d=None, speed=None, is_relative=False, wait_ok=True):
		'''Set joint angles/positions'''
		instruction = 'M21 G90'  # X{x} Y{y} Z{z} A{a} B{b} C{c} F{speed}
		if is_relative:
			instruction = 'M21 G91'
		if not speed:
			speed = self.default_speed
		if speed:
			speed = int(speed)

		pairings = {'X': x, 'Y': y, 'Z': z, 'A': a, 'B': b, 'C': c, 'D': d, 'F': speed}
		msg = self.generate_args_string(instruction, pairings)

		return self.send_msg(msg, wait_ok=wait_ok, wait_idle=True)

	def set_slider_posi(self, d, speed=None, is_relative=False, wait_ok=True):
		'''Set slider position, unit: mm'''
		if not is_relative:
			return 	self.go_to_axis(d=d,
									speed=speed, wait_ok=wait_ok)
		else:
			return 	self.go_to_axis(d=d,
									speed=speed, wait_ok=wait_ok, is_relative=True)
	
	def set_conveyor_range(self, d_min=-30000, d_max=30000):
		'''Set the displacement range of the conveyor belt'''
		# Range constraint
		if d_min < -30000:
			d_min = -30000
		if d_max > 30000:
			d_min = 30000
		# Set the maximum travel distance in the negative direction of the conveyor belt
		msg = f"$143={d_min}"
		self.send_msg(msg, wait_ok=True, wait_idle=True)
		# Set the maximum travel distance in the positive direction of the conveyor belt
		msg = f'$133={d_max}'
		self.send_msg(msg, wait_ok=True, wait_idle=True)

	def set_conveyor_posi(self, d, speed=None, is_relative=False, wait_ok=True):
		'''Set the conveyor belt position, unit: mm'''
		if not is_relative:
			return 	self.go_to_axis(d=d,
									speed=speed, wait_ok=wait_ok)
		else:
			return 	self.go_to_axis(d=d,
									speed=speed, wait_ok=wait_ok, is_relative=True)
	
	def set_tool_pose(self, x=None, y=None, z=None, roll=None, pitch=None, yaw=None, mode='p2p', speed=None, is_relative=False, wait_ok=True):
		'''Set tool pose'''
		if mode == "p2p":
			# Point-to-point mode
			self.p2p_interpolation(x=x, y=y, z=z, a=roll, b=pitch, c=yaw, speed=speed, is_relative=is_relative, wait_ok=wait_ok)
		elif mode == "linear":
			# Linear interpolation
			self.linear_interpolation(x=x, y=y, z=z, a=roll, b=pitch, c=yaw, speed=speed,is_relative=is_relative, wait_ok=wait_ok)
		else:
			# The default is point-to-point
			self.p2p_interpolation(x=x, y=y, z=z, a=roll, b=pitch, c=yaw, speed=speed, wait_ok=wait_ok)
	
	def p2p_interpolation(self, x=None, y=None, z=None, a=None, b=None, c=None, speed=None, is_relative=False, wait_ok=None):
		'''Point-to-point interpolation'''
		instruction = 'M20 G90 G0'  # X{x} Y{y} Z{z} A{a} B{b} C{c} F{speed}
		if is_relative:
			instruction = 'M20 G91 G0'

		if not speed:
			speed = self.default_speed
		if speed:
			speed = int(speed)

		pairings = {'X': x, 'Y': y, 'Z': z, 'A': a, 'B': b, 'C': c, 'F': speed}
		msg = self.generate_args_string(instruction, pairings)

		return self.send_msg(msg, wait_ok=wait_ok, wait_idle=True)

	def linear_interpolation(self, x=None, y=None, z=None, a=None, b=None, c=None, speed=None, is_relative=False, wait_ok=None):
		'''Linear interpolation'''
		instruction = 'M20 G90 G1'  # X{x} Y{y} Z{z} A{a} B{b} C{c} F{speed}
		if is_relative:
			instruction = 'M20 G91 G1'
		if not speed:
			speed = self.default_speed
		if speed:
			speed = int(speed)

		pairings = {'X': x, 'Y': y, 'Z': z, 'A': a, 'B': b, 'C': c, 'F': speed}
		msg = self.generate_args_string(instruction, pairings)
		return self.send_msg(msg, wait_ok=wait_ok, wait_idle=True)
	
	def circular_interpolation(self, ex, ey, radius, is_cw=True, speed=None, wait_ok=None):
		'''Circular interpolation
		In the XY plane, move from the current point to relative coordinates (ex, ey). 
		The radius is determined by is_cw, which determines whether the arc is clockwise or counterclockwise.
		'''
		# Check if it is legal
		distance = math.sqrt(ex**2 + ey**2)
		if distance > (radius * 2):
			self.logger.error(f'circular interpolation error, target posi is too far')
			return False

		instruction = None
		if is_cw:
			instruction = 'M20 G91 G02'
		else:
			instruction = 'M20 G91 G03'
		
		pairings = {'X': ex, 'Y': ey, 'R': radius, 'F': speed}
		msg = self.generate_args_string(instruction, pairings)
		return self.send_msg(msg, wait_ok=wait_ok, wait_idle=True)
	
	def set_door_lift_distance(self, lift_distance):
		'''Set the height at which the gantry trajectory planning is lifted'''
		msg = f"$49={lift_distance}"
		return self.send_msg(msg, wait_ok=True, wait_idle=True)

	def door_interpolation(self, x=None, y=None, z=None, a=None, b=None, c=None, speed=None, is_relative=False, wait_ok=None):
		'''Gantry interpolation'''
		instruction = 'M20 G90 G05'  # X{x} Y{y} Z{z} A{a} B{b} C{c} F{speed}
		if is_relative:
			instruction = 'M20 G91 G05'
		
		if not speed:
			speed = self.default_speed
		if speed:
			speed = int(speed)

		pairings = {'X': x, 'Y': y, 'Z': z, 'A': a, 'B': b, 'C': c, 'F': speed}
		msg = self.generate_args_string(instruction, pairings)
		return self.send_msg(msg, wait_ok=wait_ok, wait_idle=True)

	def set_tool_type(self, tool, wait_ok=True):
		'''Select tool type'''
		self.tool = tool
		self.logger.info(f"set tool {tool.name}")
		# Get the ID of the tool
		tool_id = tool.value

		if type(tool_id) != int or not (tool_id >= 0 and tool_id <= 3):
			self.logger.error(f"Unknown tool id {tool_id}")
			return False
		msg = f'$50={tool_id}'
		return self.send_msg(msg, wait_ok=wait_ok, wait_idle=True)
	
	def set_tool_offset(self, offset_x, offset_y, offset_z, wait_ok=True):
		'''Set the offset of the tool coordinate system'''
		# Set end effector x-axis offset
		msg = f"$46={offset_x}"
		ret_x = self.send_msg(msg, wait_ok=wait_ok, wait_idle=True)
		# Set end effector y-axis offset
		msg = f"$47={offset_y}"
		ret_y = self.send_msg(msg, wait_ok=wait_ok, wait_idle=True)
		# Set end effector z-axis offset
		msg = f"$48={offset_z}"
		ret_z = self.send_msg(msg, wait_ok=wait_ok, wait_idle=True)
		return ret_x and ret_y and ret_z

	def pump_suction(self):
		'''Air pump suction'''
		self.set_air_pump(self.AIR_PUMP_SUCTION_PWM_VALUE) 
	
	def pump_blowing(self):
		'''Air pump blowing'''
		self.set_air_pump(self.AIR_PUMP_BLOWING_PWM_VALUE)
	
	def pump_on(self, is_suction=True):
		"""Air pump on, suction/blowing."""
		if is_suction:
			self.set_air_pump(self.AIR_PUMP_SUCTION_PWM_VALUE)
		else:
			self.set_air_pump(self.AIR_PUMP_BLOWING_PWM_VALUE) 
	
	def pump_off(self):
		"""Air pump off, solenoid valve on, releasing air"""
		self.set_air_pump(self.AIR_PUMP_OFF_PWM_VALUE, wait_ok=False)
		self.set_valve(self.VALVE_ON_PWM_VALUE, wait_ok=False)
		time.sleep(1)
		self.set_valve(self.VALVE_OFF_PWM_VALUE, wait_ok=False)
		
	def set_air_pump(self, pwm, wait_ok=None):
		'''Set the PWM signal of the air pump'''
		if pwm not in self.pump_pwm_values:
			self.logger.exception(ValueError(f'pwm must be one of these values: {self.pump_pwm_values}. Was given {pwm}.'))
			pwm = self.AIR_PUMP_OFF_PWM_VALUE
		msg = f'M3S{pwm}'
		return self.send_msg(msg, wait_ok=wait_ok, wait_idle=True)

	def set_valve(self, pwm, wait_ok=None):
		'''Set the PWM of the solenoid valve'''
		if pwm not in self.valve_pwm_values:
			self.logger.exception(ValueError(f'pwm must be one of these values: {self.valve_pwm_values}. Was given {pwm}.'))
			pwm = self.VALVE_OFF_PWM_VALUE
		msg = f'M4E{pwm}'
		return self.send_msg(msg, wait_ok=wait_ok, wait_idle=True)
	
	def gripper_inverse_kinematic(self, spacing_mm):
		'''Claw inverse kinematics'''
		d1 = (spacing_mm / 2) + self.GRIPPER_LINK_C - self.GRIPPER_LINK_A
		theta = math.degrees(math.asin(d1/self.GRIPPER_LINK_B))
		return theta
	
	def set_gripper_spacing(self, spacing_mm):
		'''Set claw spacing'''
		# Check if it is a legal spacing constraint
		spacing_mm = max(self.GRIPPER_SPACING_MIN, min(self.GRIPPER_SPACING_MAX, spacing_mm))
		# Inverse kinematics
		theta = self.gripper_inverse_kinematic(spacing_mm)
		angle_min = self.gripper_inverse_kinematic(self.GRIPPER_SPACING_MIN)
		angle_max = self.gripper_inverse_kinematic(self.GRIPPER_SPACING_MAX)
		# Convert rotation angle to PWM value
		ratio = ((theta - angle_min) / (angle_max - angle_min))
		pwm = int(self.GRIPPER_CLOSE_PWM_VALUE + ratio * (self.GRIPPER_OPEN_PWM_VALUE - self.GRIPPER_CLOSE_PWM_VALUE))
		# print(f"Claw inverse kinematics Angle: {theta} angle_min: {angle_min} angle_max: {angle_max} PWM: {pwm}")
		# Set the claw's PWM
		self.set_gripper(pwm)
		
	def gripper_open(self):
		'''Open the claw'''
		self.set_gripper(self.GRIPPER_OPEN_PWM_VALUE)
	
	def gripper_close(self):
		'''Close the claw'''
		self.set_gripper(self.GRIPPER_CLOSE_PWM_VALUE)
	
	def set_gripper(self, pwm, wait_ok=None):
		'''Set the claw's PWM'''
		# Type constraint
		if isinstance(pwm, bool):
			if pwm == True:
				pwm = self.GRIPPER_CLOSE_PWM_VALUE
			else:
				pwm = self.GRIPPER_OPEN_PWM_VALUE
		pwm = int(pwm)
		# Numeric constraint
		lowerb = min([self.GRIPPER_OPEN_PWM_VALUE, self.GRIPPER_CLOSE_PWM_VALUE])
		upperb = max([self.GRIPPER_OPEN_PWM_VALUE, self.GRIPPER_CLOSE_PWM_VALUE])
		pwm = max(lowerb, min(upperb, pwm))
		
		msg = f'M3S{pwm}'
		return self.send_msg(msg, wait_ok=wait_ok, wait_idle=True)
	
	def start_calibration(self, wait_ok=None):
		'''Start robotic arm calibration'''
		instruction = 'M40'
		return self.send_msg(instruction, wait_ok=wait_ok)

	def finish_calibration(self, wait_ok=None):
		'''Complete robotic arm calibration'''
		instruction = 'M41'
		return self.send_msg(instruction, wait_ok=wait_ok)

	def reset_configuration(self, reset_file=None, wait_ok=None):
		'''Reset the robotic arm's configuration'''
		output = {}

		def send_each_line(file_lines):
			nonlocal output
			for line in file_lines:
				output[line] = self.send_msg(line, var_command=True, wait_ok=wait_ok)

		reset_file = reset_file if reset_file else self.reset_file

		if isinstance(reset_file, str) and '\n' in reset_file or \
		   isinstance(reset_file, bytes) and b'\n' in reset_file:
			# If we find that we have a string and it contains newlines,
			send_each_line(reset_file.splitlines())

		elif isinstance(reset_file, (str, Path)):
			if not os.path.exists(reset_file):
				self.logger.exception(MirobotResetFileError("Reset file not found or reachable: {reset_file}"))
			with open(reset_file, 'r') as f:
				send_each_line(f.readlines())

		elif isinstance(reset_file, Collection) and not isinstance(reset_file, str):
			send_each_line(reset_file)

		elif isinstance(reset_file, (TextIO, BinaryIO)):
			send_each_line(reset_file.readlines())

		else:
			self.logger.exception(MirobotResetFileError(f"Unable to handle reset file of type: {type(reset_file)}"))

		return output
	@property
	def state(self):
		'''Get Mirobot status code'''
		return self.status.state

	@property
	def pose(self):
		'''End effector pose 6dof'''
		return self.cartesian
	
	@property
	def cartesian(self):
		'''End effector pose 6dof'''
		return self.status.cartesian

	@property
	def angle(self):
		'''Joint angles'''
		return self.status.angle

	@property
	def slider(self):
		'''Get the slider (Mirobot seventh axis) position'''
		return self.status.angle.d

	@property
	def conveyor(self):
		'''Get the conveyor belt (Mirobot seventh axis) position'''
		return self.status.angle.d

	@property
	def valve_pwm(self):
		'''PWM of the solenoid valve'''
		return self.status.valve_pwm

	@property
	def pump_pwm(self):
		'''PWM of the air pump'''
		return self.status.pump_pwm

	@property
	def gripper_pwm(self):
		'''PWM of the claw'''
		return self.status.pump_pwm
	
	@property
	def motion_mode(self):
		'''Motion mode'''
		return self.status.motion_mode
