"""
Mirobot Exception Handling
"""
import logging

class ExitOnExceptionStreamHandler(logging.StreamHandler):
    """ There is an issue with the data flow; the information cannot be sent properly """
    def emit(self, record):
        super().emit(record)
        # When the log level is greater than or equal to logging.ERROR, the system will exit automatically
        if record.levelno >= logging.ERROR:
            raise SystemExit(-1)

class MirobotError(Exception):
    """ 
    An inplace class for throwing Mirobot errors. 
    MirobotError
    """
    pass

class MirobotAlarm(Exception):
    """ 
    An inplace class for throwing Mirobot alarms. 
    MirobotAlarm
    """
    pass

class MirobotReset(Exception):
    """ 
    An inplace class for when Mirobot resets.
    MirobotReset
    """
    pass

class MirobotStatusError(Exception):
    """ 
    An inplace class for when Mirobot's status message is unprocessable. 
    Mirobot's status information cannot be parsed
    """
    pass

class MirobotResetFileError(Exception):
    """ 
    An inplace class for when Mirobot has problems using the given reset file. 
    Mirobot encountered an error when trying to use the current reset configuration file
    """
    pass

class MirobotVariableCommandError(Exception):
    """ 
    An inplace class for when Mirobot finds a command that does not match variable setting-command syntax.
    Mirobot command syntax error
    """
    pass


######################################################################
## Serial port exception
######################################################################

class MirobotAmbiguousPort(Exception):
    """ 
    An inplace class for when the serial port is unconfigurable.
    Mirobot serial port configuration error
    """
    pass

class SerialDeviceReadError(Exception):
    """ 
    An inplace class for when DeviceSerial is unable to read the serial port
    Serial device read exception
    """
    pass

class SerialDeviceOpenError(Exception):
    """ 
    An inplace class for when DeviceSerial is unable to open the serial port 
    Serial device open exception
    """
    pass

class SerialDeviceCloseError(Exception):
    """ 
    An inplace class for when DeviceSerial is unable to close the serial port 
    Serial device close error
    """
    pass

class SerialDeviceWriteError(Exception):
    """ 
    An inplace class for when DeviceSerial is unable to write to the serial port 
    Serial write error
    """
    pass

######################################################################
## Bluetooth exception
######################################################################
class InvalidBluetoothAddressError(Exception):
    """ 
    An inplace class for when an invalid Bluetooth address is given 
    Bluetooth address error
    """
    pass