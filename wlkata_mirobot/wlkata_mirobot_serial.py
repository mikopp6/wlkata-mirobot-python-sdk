"""
Mirobot Serial Communication Protocol/Interface
"""
import os
import time
import serial
import logging

# Using the pyserial serial device list viewer
import serial.tools.list_ports as lp
# Exception
from .wlkata_mirobot_exceptions import MirobotError, MirobotAlarm, MirobotReset, MirobotAmbiguousPort
from .wlkata_mirobot_exceptions import ExitOnExceptionStreamHandler, SerialDeviceOpenError, SerialDeviceReadError, SerialDeviceCloseError, SerialDeviceWriteError

# The current operating system type
# posix: Linux
# nt: Windows
# java: Java Virtual Machine
os_is_nt = os.name == 'nt'
# Linux and Mac both belong to the posix standard
# posix: Portable API for Unix-like operating systems
os_is_posix = os.name == 'posix'


class DeviceSerial:
	''' Serial device, an additional layer of encapsulation on top of Serial'''
	def __init__(self, portname=None, baudrate=115200, stopbits=1, timeout=0.2, exclusive=False, debug=False):
		""" Initialization of `DeviceSerial` class
		
		Parameters
		----------
		portname : str
			 (Default value = `''`) Name of the port to connect to. (Example: 'COM3' or '/dev/ttyUSB1')
		baudrate : int
			 (Default value = `0`) Baud rate of the connection.
		stopbits : int
			 (Default value = `1`) Stopbits of the connection.
		exclusive : bool
			 (Default value = `True`) Whether to (try) forcing exclusivity of serial port for this instance. Is only a true toggle on Linux and OSx; Windows always exclusively blocks serial ports. Setting this variable to `False` on Windows will throw an error.
		debug : bool
			 (Default value = `False`) Whether to print DEBUG-level information from the runtime of this class. Show more detailed information on screen output.
		Returns
		-------
		class : DeviceSerial

		"""
		self.portname = str(portname)
		# print(f"DeviceSerial portname:  {self.portname}")
		self.baudrate = int(baudrate)
		self.stopbits = int(stopbits)
		self.timeout = int(timeout)
		self.exclusive = exclusive
		self._debug = debug

		# Initialize the logging module
		self.logger = logging.getLogger(__name__)
		self.logger.setLevel(logging.DEBUG)
		self.stream_handler = ExitOnExceptionStreamHandler()
		# Set the log level
		self.stream_handler.setLevel(logging.DEBUG if self._debug else logging.INFO)
		# Log format
		formatter = logging.Formatter(f"[{self.portname}] [%(levelname)s] %(message)s")
		self.stream_handler.setFormatter(formatter)
		self.logger.addHandler(self.stream_handler)

		if os_is_posix:
			# Set exclusive access mode (POSIX only). If the port is already open in exclusive access mode, it cannot be opened in exclusive access mode again.
			# self.serialport = serial.Serial(exclusive=exclusive)
			self.serialport = serial.Serial()
		else:
			self.serialport = serial.Serial()
		self._is_open = False

	def __del__(self):
		""" Close the serial port when the class is deleted	"""
		self.close()

	@property
	def debug(self):
		""" Return the `debug` property of `DeviceSerial` """
		return self._debug

	@debug.setter
	def debug(self, value):
		"""
		Set the new `debug` property of `DeviceSerial`. Use as in `DeviceSerial.setDebug(value)`.
		
		Parameters
		----------
		value : bool
			The new value for `DeviceSerial.debug`.
			User this setter method as it will also update the logging method. As opposed to setting `DeviceSerial.debug` directly which will not update the logger.
		"""
		self._debug = bool(value)
		self.stream_handler.setLevel(logging.DEBUG if self._debug else logging.INFO)

	@property
	def is_open(self):
		""" 
		Check if the serial port is open
		"""
		self._is_open = self.serialport.is_open
		return self._is_open

	def readline(self, timeout=0.1):
		"""
		Listen to the serial port and return a message.
		
		Parameters
		----------
		timeout : float
			Threshold for timeout, in seconds
		Returns
		-------
		msg : str
			A single line that is read from the serial port.
			TODO There is some uncertainty here because it's possible to read before receiving a newline character
		"""
		t_start = time.time()
		msg_recv = b''
		while self._is_open:
			# Timeout check
			t_cur = time.time()
			if (t_cur - t_start) >= timeout:
				# Add fault tolerance for non-UTF-8 encoded data
				return msg_recv.decode("utf-8", "ignore").strip()
			try:
				msg_seg = self.serialport.readline()
				if len(msg_seg) > 0:
					msg_recv += msg_seg
			except Exception as e:
				self.logger.exception(SerialDeviceReadError(e))
				
	def open(self):
		""" Open the serial port. """
		if not self._is_open:
			# serialport = 'portname', baudrate, bytesize = 8, parity = 'N', stopbits = 1, timeout = None, xonxoff = 0, rtscts = 0)
			# Configure the parameters of the serial port:
			# - Port number portname
			# - Baud rate baudrate
			# - Stop bits stopbits
			# - Timeout wait timeout
			self.serialport.port = self.portname
			self.serialport.baudrate = self.baudrate
			self.serialport.stopbits = self.stopbits
			self.serialport.timeout = self.timeout
			
			try:
				self.logger.debug(f"Welcome to Wlkata Mirobot Python SDK")
				self.logger.debug(f"- Attempting to open serial port {self.portname}")
				self.serialport.open()
				self._is_open = True
				self.logger.debug(f"- Succesfully open serial port {self.portname}")
				self.logger.debug("Wait 1s")
				time.sleep(1)
	
				return True
			except Exception as e:
				self.logger.exception(SerialDeviceOpenError(e))
				return False
	def close(self):
		""" Close the serial port """
		if self._is_open:
			try:
				self.logger.debug(f"Attempting to close serial port {self.portname}")

				self._is_open = False
				self.serialport.close()

				self.logger.debug(f"Succeeded in closing serial port {self.portname}")

			except Exception as e:
				self.logger.exception(SerialDeviceCloseError(e))

	def send(self, message, terminator=os.linesep):
		"""
		Send a message to the serial port.

		Parameters
		----------
		message : str
			The string to send to serial port.
		terminator : str
			(Default value = `os.linesep`) The line separator to use when signaling a new line. Usually `'\\r\\n'` for windows and `'\\n'` for modern operating systems.

		Returns
		-------
		result : bool
			Whether the sending of `message` succeeded.

		"""
		if self._is_open:
			try:
				# Automatically add line breaks
				if not message.endswith(terminator):
					message += terminator
				# Serial data transmission, encoded in utf-8
				self.serialport.write(message.encode('utf-8'))
			except Exception as e:
				# Log write to serial device write exception
				self.logger.exception(SerialDeviceWriteError(e))

			else:
				return True
		else:
			return False


class WlkataMirobotSerial:
	""" A class for bridging the interface between `mirobot.wlkata_mirobot_gcode_protocol.WlkataMirobotGcodeProtocol` and `mirobot.serial_device.DeviceSerial`"""
	def __init__(self, mirobot, portname=None, baudrate=None, stopbits=None, exclusive=True, debug=False, logger=None, autofindport=True):
		''' Mirobot serial communication interface'''
		# self.logger.info(f"WlkataMirobotSerial port number: {portname}")
		self.mirobot = mirobot
		if logger is not None:
			self.logger = logger

		self._debug = debug
		serial_device_kwargs = {'debug': debug, 'exclusive': exclusive}
		
		# Check if baudrate was passed in args or kwargs, if not, use the default value instead
		if baudrate is None:
			# Set the default baud rate
			serial_device_kwargs['baudrate'] = 115200
		# Check if stopbits was passed in args or kwargs, if not, use the default value instead
		if stopbits is None:
			# Set the default stop bit configuration
			serial_device_kwargs['stopbits'] = 1

		# If portname was not passed in and autofindport is set to true, autosearch for a serial port
		# If the port number is not specified, search automatically
		if portname is not None:
			# Set the port number
			self.default_portname = portname
			serial_device_kwargs['portname'] = portname
		elif autofindport and portname is None:
			self.default_portname = self._find_portname()
			""" The default portname to use when making connections. To override this on a individual basis, provide portname to each invokation of `WlkataMirobotGcodeProtocol.connect`. """
			serial_device_kwargs['portname'] = self.default_portname
			self.logger.info(f"Using Serial Port \"{self.default_portname}\"")
		
		# Create a serial device
		# print("Create serial object DeviceSerial")
		self.serial_device = DeviceSerial(**serial_device_kwargs)
		# Open the serial port
		self.serial_device.open()
		# Reset the robot
		# Force restart
		self.serial_device.serialport.write("%\n".encode("utf-8"))
	
	@property
	def debug(self):
		""" Return the `debug` property of `WlkataMirobotSerial` """
		return self._debug

	@debug.setter
	def debug(self, value):
		''' Set the debug switch'''
		self._debug = bool(value)
		self.serial_device.setDebug(value)

	def send(self, msg, disable_debug=False, terminator=os.linesep, wait_ok=False, wait_idle=False):
		''' Send commands to Mirobot'''
		# Clear the buffer before sending a message
		cache_msg = self.empty_cache()
		if self._debug and not disable_debug:
			# Print buffered data
			if len(cache_msg) != 0:
				self.logger.debug(f"[RECV CACHE] {cache_msg}")

		output = self.serial_device.send(msg, terminator=terminator)
		
		if self._debug and not disable_debug:
			self.logger.debug(f"[SENT] {msg}")

		if wait_ok is None:
			wait_ok = False
		
		if wait_idle is None:
			wait_idle = False
			
		if wait_ok:
			output = self.wait_for_ok(disable_debug=disable_debug)
		
		if wait_idle:
			self.wait_until_idle()
		
		return output
	
	@property
	def is_connected(self):
		''' Is the serial port connected?'''
		return self.serial_device.is_open
	
	def _is_mirobot_device(self, portname):
		''' Is it a Mirobot device?'''
		self.logger.info(f"Attempting to open serial port {portname}")
		# Attempt to open the device
		try:
			device=serial.Serial(portname, 115200, timeout=0.1)
		except Exception as e:
			self.logger.error(e)
			return False
		
		if not device.isOpen():
			self.logger.error("Serial is not open")
			return False
		# device.write("?\n".encode("utf-8"))
		# Device reset, compatible with the sub-control board
		device.write("%\n".encode("utf-8"))
		time.sleep(1)
		# Read all characters and check if 'WLKATA' is in the received string
		# Add fault tolerance for non-UTF-8 encoded data
		recv_str = device.readall().decode('utf-8', "ignore")
		self.logger.info(f"[RECV] {recv_str}")
		is_mirobot = 'WLKATA' in recv_str or 'Qinnew' in recv_str
		# Close the device
		device.close()
		return is_mirobot
	
	def _find_portname(self):
		''' Automatically detect possible Mirobot port numbers'''
		port_objects = lp.comports()

		if not port_objects:
			self.logger.exception(MirobotAmbiguousPort("No ports found! Make sure your Mirobot is connected and recognized by your operating system."))
		else:
			for p in port_objects:
				# Try to establish a connection, send a command, and see if you can get a response
				if self._is_mirobot_device(p.device):
					return p.device
			self.logger.exception(MirobotAmbiguousPort("No open ports found! Make sure your Mirobot is connected and is not being used by another process."))

	def wait_for_ok(self, reset_expected=False, disable_debug=False):
		''' Wait for an "ok" response'''
		output = ['']
		# Represents the ok suffix
		ok_eols = ['ok']
		# Reset reset character
		reset_strings = ['Using reset pos!']

		# eol: end of line
		def matches_eol_strings(terms, s):
			# print("matches_eol_strings: s={}".format(s))
			for eol in terms:
				# Changed the condition for ok judgment here
				# Because after a successful homing, it returns "homeing moving...ok" instead of just "ok"
				# Optimized for this situation to prevent getting stuck
				if s.endswith(eol) or eol in s:
					# self.logger.debug(f'String {s} terms:{terms}, match')
					return True
			return False

		if reset_expected:
			eols = ok_eols + reset_strings
		else:
			eols = ok_eols

		if os_is_nt and not reset_expected:
			# Expected number of ok returns for Windows
			# eol_threshold = 2  # It seems like the author made a mistake here
			eol_threshold = 1
		else:
			# Expected number of ok returns for Linux
			eol_threshold = 1

		eol_counter = 0
		while eol_counter < eol_threshold:
			# Read the message
			# The issue here is that this listen_to_device is an infinite loop
			msg = self.serial_device.readline(timeout=0.1)
			# Debug, print received messages
			if self._debug and not disable_debug:
				if len(msg) != 0:
					self.logger.debug(f"[RECV] {msg}")
			# Exception handling
			if 'error' in msg:
				self.logger.error(MirobotError(msg.replace('error: ', '')))
			# Exception handling
			if 'ALARM' in msg:
				self.logger.error(MirobotAlarm(msg.split('ALARM: ', 1)[1]))

			output.append(msg)

			if not reset_expected and matches_eol_strings(reset_strings, msg):
				self.logger.error(MirobotReset('Mirobot was unexpectedly reset!'))

			if matches_eol_strings(eols, output[-1]):
				eol_counter += 1

		return output[1:]  # Don't include the dummy empty string at the first index

	def wait_until_idle(self, refresh_rate=0.1):
		''' Wait until the system status is Idle'''
		# Update the current state of Mirobot
		self.mirobot.get_status(disable_debug=True)
		while self.mirobot.status is None or self.mirobot.status.state != 'Idle':
			time.sleep(refresh_rate)
			# Continuously send status queries to update the state
			self.mirobot.get_status(disable_debug=True)
	def empty_cache(self):
		''' Clear the receive buffer'''
		cache_msg = ""
		while(self.serial_device.serialport.in_waiting):
			# Add fault tolerance for non-UTF-8 encoded data
			cache_msg += self.serial_device.serialport.read().decode('utf-8', 'ignore')
		return cache_msg

	def connect(self, portname=None):
		''' Establish a serial connection'''
		if portname is None:
			if self.default_portname is not None:
				portname = self.default_portname
			else:
				self.logger.exception(ValueError('Portname must be provided! Example: `portname="COM3"`'))

		self.serial_device.portname = portname
		self.serial_device.open()
		# return self.wait_for_ok(reset_expected=True)
		return True
	
	def disconnect(self):
		''' Disconnect from Mirobot, close the serial port'''
		if getattr(self, 'serial_device', None) is not None:
			self.serial_device.close()
