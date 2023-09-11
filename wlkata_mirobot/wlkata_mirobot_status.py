"""
Mirobot's status information includes:
- `MirobotAngles`: Robotic arm joint angles
- `MirobotCartesians`: Robotic arm end effector pose in Cartesian coordinates
- `MirobotStatus`: Robotic arm system status
"""
from dataclasses import dataclass, asdict, astuple, fields
import numbers
import operator


class BasicDataClass:
    def asdict(self):
        return asdict(self)

    def astuple(self):
        return astuple(self)

    def fields(self):
        return fields(self)

    @classmethod
    def _new_from_dict(cls, dictionary):
        return cls(**dictionary)

class FeaturedDataClass(BasicDataClass):
    def _cross_same_type(self, other, operation_function, single=False):
        new_values = {}
        for f in self.fields():
            this_value = getattr(self, f.name)

            if single:
                other_value = other
            else:
                other_value = getattr(other, f.name)

            result = operation_function(this_value, other_value)

            new_values[f.name] = result

        return new_values

    def _binary_operation(self, other, operation):
        def operation_function(this_value, other_value):
            if None in (this_value, other_value):
                return None
            else:
                return operation(this_value, other_value)

        if isinstance(other, type(self)):
            new_values = self._cross_same_type(other, operation_function)

        elif isinstance(other, numbers.Real):
            new_values = self._cross_same_type(other, operation_function, single=True)

        else:
            raise NotImplementedError(f"Cannot handle {type(self)} and {type(other)}")

        return self._new_from_dict(new_values)

    def _unary_operation(self, operation_function):
        new_values = {f.name: operation_function(f)
                      for f in self.fields()}

        return self._new_from_dict(new_values)

    def _basic_unary_operation(self, operation):
        def operation_function(field):
            value = getattr(self, field.name)
            if value is not None:
                return operation(value)
            else:
                return None

        return self._unary_operation(operation_function)

    def _comparision_operation(self, other, operation):
        def operation_function(this_value, other_value):
            if None in (this_value, other_value):
                return True
            else:
                return operation(this_value, other_value)

        if isinstance(other, type(self)):
            new_values = self._cross_same_type(other, operation_function).values()

        elif isinstance(other, (int, float)):
            new_values = self._cross_same_type(other, operation_function, single=True).values()

        else:
            raise NotImplementedError(f"Cannot handle {type(self)} and {type(other)}")

        if all(new_values):
            return True

        elif not any(new_values):
            return False

        else:
            return None

    def __or__(self, other):
        def operation_function(this_value, other_value):
            if this_value is None:
                return other_value
            else:
                return this_value

        new_values = self._cross_same_type(other, operation_function)
        return self._new_from_dict(new_values)

    def __and__(self, other):
        def operation_function(this_value, other_value):
            if None not in (this_value, other_value):
                return this_value
            else:
                return None

        new_values = self._cross_same_type(other, operation_function)
        return self._new_from_dict(new_values)

    def int(self):
        def operation_function(field):
            value = getattr(self, field.name)
            if field.type in (float,) and value is not None:
                return int(value)
            else:
                return value

        return self._unary_operation(operation_function)

    def round(self):
        def operation_function(field):
            value = getattr(self, field.name)
            if field.type in (float,) and value is not None:
                return round(value)
            else:
                return value

        return self._unary_operation(operation_function)

    def __add__(self, other):
        return self._binary_operation(other, operator.add)

    def __radd__(self, other):
        return self._binary_operation(other, operator.add)

    def __sub__(self, other):
        return self._binary_operation(other, operator.sub)

    def __rsub__(self, other):
        def rsub(dataclass_value, number):
            return operator.sub(number, dataclass_value)

        return self._binary_operation(other, rsub)

    def __mul__(self, other):
        return self._binary_operation(other, operator.mul)

    def __rmul__(self, other):
        return self._binary_operation(other, operator.mul)

    def __div__(self, other):
        return self._binary_operation(other, operator.div)

    def __rdiv__(self, other):
        def rdiv(dataclass_value, number):
            return operator.div(number, dataclass_value)

        return self._binary_operation(other, rdiv)

    def __truediv__(self, other):
        return self._binary_operation(other, operator.truediv)

    def __rtruediv__(self, other):
        def rtruediv(dataclass_value, number):
            return operator.truediv(number, dataclass_value)

        return self._binary_operation(other, operator.truediv)

    def __mod__(self, other):
        return self._binary_operation(other, operator.mod)

    def __abs__(self):
        return self._basic_unary_operation(operator.abs)

    def __pos__(self):
        return self._basic_unary_operation(operator.pos)

    def __neg__(self):
        return self._basic_unary_operation(operator.neg)

    def __lt__(self, other):
        return self._comparision_operation(other, operator.lt)

    def __le__(self, other):
        return self._comparision_operation(other, operator.le)

    def __eq__(self, other):
        return self._comparision_operation(other, operator.eq)

    def __ne__(self, other):
        return self._comparision_operation(other, operator.ne)

    def __ge__(self, other):
        return self._comparision_operation(other, operator.ge)

    def __gt__(self, other):
        return self._comparision_operation(other, operator.gt)

@dataclass
class MirobotAngles(FeaturedDataClass):
    """
    Mirobot joint angles
    """
    a: float = None # Joint 1 angle
    b: float = None # Joint 2 angle
    c: float = None # Joint 3 angle
    x: float = None # Joint 4 angle
    y: float = None # Joint 5 angle
    z: float = None # Joint 6 angle
    d: float = None # Position of the seventh axis slider
    @property
    def a1(self):
        """ Joint 1 angle, unit: ° """
        return self.a

    @property
    def a2(self):
        """ Joint 2 angle, unit: ° """
        return self.b

    @property
    def a3(self):
        """ Joint 3 angle, unit: ° """
        return self.c

    @property
    def a4(self):
        """ Joint 4 angle, unit: ° """
        return self.x

    @property
    def a5(self):
        """ Joint 5 angle, unit: ° """
        return self.y

    @property
    def a6(self):
        """ Joint 6 angle, unit: ° """
        return self.z

    @property
    def rail(self):
        """ The seventh axis, which is the linear slider's translation """
        return self.d

    @property
    def joint1(self):
        """ Joint 1 angle, unit: ° """
        return self.x

    @property
    def joint2(self):
        """ Joint 2 angle, unit: ° """
        return self.y

    @property
    def joint3(self):
        """ Joint 3 angle, unit: ° """
        return self.z

    @property
    def joint4(self):
        """ Joint 4 angle, unit: ° """
        return self.a
    
    @property
    def joint5(self):
        """ Joint 5 angle, unit: ° """
        return self.b
    
    @property
    def joint6(self):
        """ Joint 6 angle, unit: ° """
        return self.c

@dataclass
class MirobotCartesians(FeaturedDataClass):
    """ 
    Pose in Cartesian coordinates, including:
    - End effector position: (ex, ey, ez)
    - End effector Euler angles: (roll, pitch, yaw)
    """
    x: float = None # End effector X-coordinate, unit: mm
    y: float = None # End effector Y-coordinate, unit: mm
    z: float = None # End effector Z-coordinate, unit: mm
    a: float = None # Roll angle, unit: degrees
    b: float = None # Pitch angle, unit: degrees
    c: float = None # Yaw angle, unit: degrees

    @property
    def tx(self):
        """ End effector X-coordinate, unit: mm """
        return self.x

    @property
    def ty(self):
        """ End effector Y-coordinate, unit: mm """
        return self.y

    @property
    def tz(self):
        """ End effector Z-coordinate, unit: mm """
        return self.z

    
    @property
    def rx(self):
        """ Roll angle, unit: degrees """
        return self.a

    @property
    def ry(self):
        """ Pitch angle, unit: degrees """
        return self.b

    @property
    def rz(self):
        """ Yaw angle, unit: degrees """
        return self.c

    @property
    def roll(self):
        """ Roll angle, unit: degrees """
        return self.a
    
    @property
    def pitch(self):
        """ Pitch angle, unit: degrees """
        return self.b
    
    @property
    def yaw(self):
        """ Yaw angle, unit: degrees """
        return self.c
    
    def __str__(self):
        return f"Pose(x={self.x},y={self.y},z={self.z},roll={self.roll},pitch={self.pitch},yaw={self.yaw})"
        
@dataclass
class MirobotStatus(BasicDataClass):
    """ 
    A composite dataclass to hold all of Mirobot's trackable quantities.
    Mirobot status information is stored using a composite data structure.
    """
    # Mirobot status string
    state: str = ''
    # Record joint angle information
    angle: MirobotAngles = MirobotAngles()
    # Store pose information in Cartesian coordinates (xyz coordinates and rpy angles)
    cartesian: MirobotCartesians = MirobotCartesians()
    # PWM for the air pump
    pump_pwm: int = None
    # PWM for the solenoid valve/claw
    valve_pwm: int = None
    # Current motion mode
    # False: Cartesian coordinate mode
    # True: Joint motion mode
    motion_mode: bool = False