# (C) 2016  Jean Nassar
# Released under the GNU General Public License, version 3
"""
Module for interfacing with Dynamixels (Dx).

Currently supported Dynamixel types are:

    - AX-12
    - MX-28
"""
from collections import ChainMap
import serial
import time


class DynamixelError(Exception):
    """Custom exception class"""


class Dynamixel(object):
    """
    A parent Dynamixel class which contains common functions.

    Parameters
    ----------
    dx_id : int
        The current ID of the Dx.
    dx_type : str, optional
        The type of the Dx. Default is "Dynamixel".
    name : str, optional
        The name of the Dx. Default is "Dynamixel".
    port : str, optional
        The port to which the Dx is connected. Default is "/dev/ttyUSB0".
        Windows users can use COM ports instead.
    baudrate : int, optional
        The communication baudrate of the Dx. Default is 1,000,000.
    timeout : int, optional
        The length of time, in seconds, after which the communication with the
        Dx is interrupted if no response is received. Default is 5.
        seconds.

    Attributes
    ----------
    dx_type : str
        The type of the Dx.
    name : str
        The name of the Dx.
    port : str
        The port to which the Dx is connected.
    ser : Serial
        The serial device to communicate with the Dx.
    errors : dict
        Contains all error bits and their meanings.

        **Dictionary format :** {id (int): name (str)}
    dx_ids : list of int
        Contains the IDs of all Dx currently connected.

    Raises
    ------
    DynamixelError
        If the ID is wrong or is already registered.

    """
    class Response(object):
        """
        A response object for communicating with the Dx.

        The response object is used to validate the data for safety purposes.

        Parameters
        ----------
        data : list of byte
            The data received from the Dx.

        Attributes
        ----------
        data : list of byte
            The data received from the Dx.
        response_id : byte
            The ID of the response.
        length : byte
            The length of the response.
        errors : list of str
            The errors raised by the response.
        value : int
            The value returned by the data.

        Raises
        ------
        DynamixelError
            If the data received is bad, or if there are errors.

        """
        def __init__(self, data):
            # Check data integrity
            if not data or 0xff not in data[:2]:
                raise DynamixelError("Bad Header! ({data})".format(data=data))
            if Dynamixel._checksum(data[2:-1]) != data[-1]:
                raise DynamixelError("Checksum {chk} should be {good}!".format(
                    chk=Dynamixel._checksum(data[2:-1]), good=data[-1]))

            self.data = data
            self.response_id, self.length = data[2:4]
            self.errors = []

            # Check for errors
            for k in Dynamixel.errors.keys():
                if data[4] & k != 0:
                    self.errors.append(Dynamixel.errors[k])
            if self.errors:
                raise DynamixelError("ERRORS: {e}".format(e=self.errors))

            # Retrieve data
            requested_data = self.data[5:-1]

            if not requested_data:
                return
            elif len(requested_data) == 1:
                self.value = requested_data[0]
            elif len(requested_data) == 2:  # Little endian
                self.value = (requested_data[1] << 8) + requested_data[0]
            else:
                raise DynamixelError("Bad number of values returned!")

        def __str__(self):
            """String representation of the data."""
            return " ".join([hex(i) for i in self.data])

    # Instructions that can be sent to the Dynamixel
    _instructions = {
        "ping": 0x01,
        "read_data": 0x02,
        "write_data": 0x03,
        "reg_write": 0x04,
        "action": 0x05,
        "reset": 0x06,
        "sync_write": 0x83
    }

    # Registers common to all Dynamixel types
    _registers = {
        "model_number": 0x00,
        "firmware_version": 0x02,
        "id": 0x03,
        "baudrate": 0x04,
        "return_delay": 0x05,
        "cw_limit": 0x06,
        "ccw_limit": 0x08,
        "max_temperature": 0x0B,
        "min_voltage": 0x0C,
        "max_voltage": 0x0D,
        "max_torque": 0x0E,
        "status_return_level": 0x10,
        "alarm_led": 0x11,
        "alarm_shutdown": 0x12,
        "torque_enable": 0x18,
        "led": 0x19,
        "goal_position": 0x1E,
        "moving_speed": 0x20,
        "torque_limit": 0x22,
        "present_position": 0x24,
        "present_speed": 0x26,
        "present_load": 0x28,
        "present_voltage": 0x2A,
        "present_temperature": 0x2B,
        "registered_instruction": 0x2C,
        "moving": 0x2E,
        "lock": 0x2F,
        "punch": 0x30
    }

    # Minimum limits of registers
    _register_minima = {}

    # Maximum limits of registers
    _register_maxima = {
        "id": 253,
        "baudrate": 254,
        "return_delay": 254,
        "cw_limit": 1023,
        "ccw_limit": 1023,
        "max_temperature": 150,
        "min_voltage": 250,
        "max_voltage": 250,
        "max_torque": 1023,
        "status_return_level": 2,
        "alarm_led": 127,
        "alarm_shutdown": 127,
        "torque_enable": 1,
        "led": 1,
        "goal_position": 1023,
        "moving_speed": 1023,
        "torque_limit": 1023,
        "registered_instruction": 1,
        "lock": 1,
        "punch": 1023
    }

    # Registers that are two bytes long
    _two_byte_registers = {
        "model_number",
        "cw_limit",
        "ccw_limit",
        "max_torque",
        "goal_position",
        "moving_speed",
        "torque_limit",
        "present_position",
        "present_speed",
        "present_load",
        "punch"
    }

    # Error bits
    errors = {
        1:  "InputVoltage",
        2:  "AngleLimit",
        4:  "Overheating",
        8:  "Range",
        16: "Checksum",
        32: "Overload",
        64: "Instruction"
    }

    dx_ids = []

    def __init__(self, dx_id, dx_type="Dynamixel", name="Dynamixel",
                 port="/dev/ttyUSB0", baudrate=1000000, timeout=5):
        self._verify_id(dx_id)
        self.dx_type = dx_type
        self._dx_id = dx_id
        self.name = name
        self.port = port
        self.ser = serial.Serial(self.port, baudrate=baudrate, timeout=timeout)

        try:
            self.dx_id
        except DynamixelError:
            raise DynamixelError("ID# {dx_id} Dynamixel not connected.".format(
                dx_id=dx_id))

        Dynamixel.dx_ids.append(dx_id)

    def _interact(self, packet):
        """
        Communicate with the Dx.

        Parameters
        ----------
        packet : list of byte
            The packet to send to the Dx.

        Returns
        -------
        Response
            The response from the Dx.

        """
        payload = [self._dx_id, len(packet)+1] + packet
        to_write = [0xFF, 0xFF] + payload + [Dynamixel._checksum(payload)]
        self.ser.write(bytes(to_write))
        time.sleep(0.05)

        res = self.ser.read(self.ser.inWaiting())
        return Dynamixel.Response([i for i in res])

    def read(self, register):
        """
        Read the value of a register.

        Parameters
        ----------
        register : str
            The name of the register to be read.

        Returns
        -------
        int
            The value of the register.

        """
        packet = []
        packet.append(self._instructions["read_data"])
        packet.append(self._registers[register])
        packet.append(self._register_length(register))
        return self._interact(packet).value

    def write(self, register, value):
        """
        Write a value to a register.

        Parameters
        ----------
        register : str
            The name of the register to be written to.
        value : int
            The value to write to the register.

        Raises
        ------
        DynamixelError
            If the value supplied is illegal.

        """
        length = self._register_length(register)

        if register in self._register_minima:
            min_value = self._register_minima[register]
        else:
            min_value = 0
        max_value = self._register_maxima[register]
        if not min_value <= value <= max_value:
            raise DynamixelError("Illegal value: {v}".format(v=value))

        value = int(value)

        packet = []
        packet.append(self._instructions["write_data"])
        packet.append(self._registers[register])
        if length == 1:
            packet.append(value)
        else:
            packet.append(value & 255)
            packet.append(value >> 8)
        self._interact(packet)

    def reset(self):
        """
        Send a reset command to the Dx.

        Resetting also changes the Dx ID to 1.

        Raises
        ------
        DynamixelError
            If resetting would cause a conflict.

        """
        if 1 in Dynamixel.dx_ids and self.dx_id != 1:
            raise DynamixelError("Resetting would conflict with Dynamixel #1.")
        current_id = self.dx_id
        self._interact([self._instructions["reset"]])
        time.sleep(0.25)
        self._dx_id = 1
        self.dx_id = current_id

    @property
    def model_number(self):
        """Get the model number."""
        return self.read("model_number")

    @property
    def firmware_version(self):
        """Get the firmware version."""
        return self.read("firmware_version")

    @property
    def dx_id(self):
        """Get the Dx ID."""
        return self.read("id")

    @dx_id.setter
    def dx_id(self, new_id):
        """Set the Dx ID."""
        self._verify_id(new_id)
        old_id = self.dx_id
        self.write("id", new_id)
        self._dx_id = new_id
        Dynamixel.dx_ids.remove(old_id)
        Dynamixel.dx_ids.append(new_id)

    @property
    def baudrate(self):
        """Get the baudrate."""
        return 2000000 / (self.read("baudrate") + 1)

    @baudrate.setter
    def baudrate(self, baudrate):
        """Set the baudrate."""
        value = 2000000 / baudrate - 1
        self.write("baudrate", value)

    @property
    def return_delay(self):
        """Get the return delay, in microseconds."""
        return 2 * self.read("return_delay")

    @return_delay.setter
    def return_delay(self, value):
        """Set the return delay, in microseconds."""
        self.write("return_delay", value / 2)

    @property
    def cw_limit_raw(self):
        """Get the raw value of the clockwise limit register."""
        return self.read("cw_limit")

    @cw_limit_raw.setter
    def cw_limit_raw(self, limit):
        """Set the raw value of the clockwise limit register."""
        self.write("cw_limit", limit)

    @property
    def cw_limit(self):
        """Get the clockwise limit, in degrees."""
        alpha = self._max_turn_angle / self._register_maxima["cw_limit"]
        return self.cw_limit_raw * alpha

    @cw_limit.setter
    def cw_limit(self, deg):
        """Set the clockwise limit, in degrees."""
        alpha = self._max_turn_angle / self._register_maxima["cw_limit"]
        self.cw_limit_raw = deg / alpha

    @property
    def ccw_limit_raw(self):
        """Get the raw value of the counterclockwise limit register."""
        return self.read("ccw_limit")

    @ccw_limit_raw.setter
    def ccw_limit_raw(self, limit):
        """Set the raw value of the counterclockwise limit register."""
        self.write("ccw_limit", limit)

    @property
    def ccw_limit(self):
        """Get the counterclockwise limit, in degrees."""
        alpha = self._max_turn_angle / self._register_maxima["ccw_limit"]
        return self.ccw_limit_raw * alpha

    @ccw_limit.setter
    def ccw_limit(self, deg):
        """Set the counterclockwise limit, in degrees."""
        alpha = self._max_turn_angle / self._register_maxima["ccw_limit"]
        self.ccw_limit_raw = deg / alpha

    @property
    def limits(self):
        """Get the clockwise and counterclockwise limits, in degrees."""
        return self.cw_limit, self.ccw_limit

    @limits.setter
    def limits(self, limits):
        """Set the clockwise and counterclockwise limits, in degrees."""
        try:
            cw_limit = limits[0]
            ccw_limit = limits[1]
        except TypeError:
            cw_limit = ccw_limit = limits

        if cw_limit is not None:
            self.cw_limit = cw_limit
        if ccw_limit is not None:
            self.ccw_limit = ccw_limit

    @property
    def continuous_rotation(self):
        """True if continous rotation mode is enabled."""
        return not self.cw_limit and not self.ccw_limit

    def engage_continuous_rotation(self):
        """Enable continous rotation mode."""
        self.limits = (0, 0)

    @property
    def max_temperature(self):
        """Get the maximum operating temperature, in degrees celsius."""
        return self.read("max_temperature")

    @max_temperature.setter
    def max_temperature(self, value):
        """Set the maximum operating temperature, in degrees celsius."""
        self.write("max_temperature", value)

    @property
    def min_voltage(self):
        """Get the minimum operating voltage."""
        return self.read("min_voltage") / 10

    @min_voltage.setter
    def min_voltage(self, value):
        """Set the minimum operating voltage."""
        self.write("min_voltage", value * 10)

    @property
    def max_voltage(self):
        """Get the maximum operating voltage."""
        return self.read("max_voltage") / 10

    @max_voltage.setter
    def max_voltage(self, value):
        """Set the maximum operating voltage."""
        self.write("max_voltage", value * 10)

    @property
    def max_torque(self):
        """Get the maximum operating torque."""
        return self.read("max_torque")

    @max_torque.setter
    def max_torque(self, value):
        """Set the maximum operating torque."""
        self.write("max_torque", value)

    @property
    def status_return_level(self):
        """Get the status return level."""
        return self.read("status_return_level")

    @status_return_level.setter
    def status_return_level(self, value):
        """Set the status return level."""
        self.write("status_return_level", value)

    @property
    def alarm_led(self):
        """Read the alarm LED register."""
        return self.read("alarm_led")

    @alarm_led.setter
    def alarm_led(self, value):
        """Write to the alarm LED register."""
        self.write("alarm_led", value)

    @property
    def alarm_shutdown(self):
        """Read the alarm shutdown register."""
        return self.read("alarm_shutdown")

    @alarm_shutdown.setter
    def alarm_shutdown(self, value):
        """Write to the alarm shutdown register."""
        self.write("alarm_shutdown", value)

    @property
    def torque_enable(self):
        """1 if torque is enabled."""
        return self.read("torque_enable")

    @torque_enable.setter
    def torque_enable(self, value):
        """Enable or disable the torque."""
        self.write("torque_enable", value)

    @property
    def led(self):
        """1 if the LED is on."""
        return self.read("led")

    @led.setter
    def led(self, value):
        """Turn the LED on or off."""
        self.write("led", value)

    @property
    def goal_raw(self):
        """Get the raw value of the goal position register."""
        return self.read("goal_position")

    @goal_raw.setter
    def goal_raw(self, goal):
        """Set the raw value of the goal position register."""
        self.write("goal_position", goal)

    @property
    def goal(self):
        """Get the goal position, in degrees."""
        alpha = self._max_turn_angle / self._register_maxima["goal_position"]
        return self.goal_raw * alpha

    @goal.setter
    def goal(self, deg):
        """Set the goal position, in degrees."""
        alpha = self._max_turn_angle / self._register_maxima["goal_position"]
        self.goal_raw = deg / alpha

    @property
    def moving_speed(self):
        """Get the raw value of the moving speed register."""
        return self.read("moving_speed")

    @moving_speed.setter
    def moving_speed(self, speed):
        """Set the raw value of the moving speed register."""
        self.write("moving_speed", speed)

    @property
    def moving_speed_rpm(self):
        """Get the moving speed, in RPM."""
        return self.moving_speed * 114 / 1023

    @moving_speed_rpm.setter
    def moving_speed_rpm(self, rpm):
        """Set the moving speed, in RPM."""
        self.moving_speed = rpm * 1023 / 114

    @property
    def torque_limit(self):
        """Read the torque limit register."""
        return self.read("torque_limit")

    @torque_limit.setter
    def torque_limit(self, torque_limit):
        """Write to the torque limit register."""
        self.write("torque_limit", torque_limit)

    @property
    def position_raw(self):
        """Get the raw value of the position register."""
        return self.read("present_position")

    @property
    def position(self):
        """Get the position in degrees."""
        alpha = self._max_turn_angle / self._register_maxima["goal_position"]
        return self.position_raw * alpha

    @property
    def speed(self):
        """Get the raw value of the speed register."""
        return self.read("present_speed")

    @property
    def speed_rpm(self):
        """Get the speed in RPM."""
        speed = self.speed
        direction = speed >> 10
        return (speed & 1023) * 114 / 1023 * (1 if direction else -1)

    @property
    def load(self):
        """Read the load register."""
        load = self.read("present_load")
        direction = load >> 10
        return load * (1 if direction else -1)

    @property
    def voltage(self):
        """Get the voltage, in Volts."""
        return self.read("present_voltage") / 10

    @property
    def temperature(self):
        """Get the temperature, in degrees celsius."""
        return self.read("present_temperature")

    @property
    def is_moving(self):
        """True if the servo is currently moving."""
        return self.read("moving")

    @property
    def lock(self):
        """True if the servo is locked."""
        return self.read("lock")

    @lock.setter
    def lock(self, value):
        """Lock or unlock the servo."""
        self.write("lock", value)

    @property
    def punch(self):
        """Read the punch register."""
        return self.read("punch")

    @punch.setter
    def punch(self, value):
        """Write to the punch register."""
        self.write("punch", value)

    def wait_until_stopped(self):
        """Do not send any commands until the Dx stops moving."""
        while self.is_moving:
            pass

    def _register_length(self, register):
        """Return the length of the register."""
        return 2 if register in self._two_byte_registers else 1

    def _verify_id(self, dx_id):
        """
        Verify a Dynamixel ID.

        The ID needs to be within the proper range, and a Dx with that ID must
        not be currently registered.

        Parameters
        ----------
        dx_id : int
            The ID to verify.

        Raises
        ------
        DynamixelError
            If the ID cannot be used.

        """
        if not 0 <= dx_id <= self._register_maxima["id"]:
            raise DynamixelError("ID {dx_id} is not legal!".format(dx_id=dx_id))

        if dx_id in Dynamixel.dx_ids:
            raise DynamixelError("ID# {dx_id} is already registered!".format(
                dx_id=dx_id))

    @staticmethod
    def _checksum(s):
        """
        Perform a checksum on a list of bytes.

        Parameters
        ----------
        s : list of byte
            The list of bytes to be checked.

        Return
        ------
        The checksum of `s`.

        """
        return (~sum(s)) & 0xFF

    def close(self):
        """Close the connection to a Dx."""
        Dynamixel.dx_ids.remove(self.dx_id)
        self.ser.close()

    def __enter__(self):
        """Context manager entry."""
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        """Context manager exit."""
        self.close()

    def __del__(self):
        """Deletion."""
        self.close()

    def __repr__(self):
        """Representation."""
        return "{dx_type} Dynamixel (ID# {dx_id}): {name}".format(
            dx_type=self.dx_type, dx_id=self.dx_id, name=self.name)

    def __str__(self):
        """String representation."""
        return self.name


class AX12(Dynamixel):
    """
    The AX-12 series of Dx.

    Parameters
    ----------
    dx_id : int
        The current ID of the Dx.
    name : str, optional
        The name of the Dx. Default is "AX-12".
    port : str, optional
        The port to which the Dx is connected. Default is "/dev/ttyUSB0".
        Windows users can use COM ports instead.
    baudrate : int, optional
        The communication baudrate of the Dx. Default is 1,000,000.
    timeout : int, optional
        The length of time, in seconds, after which the communication with the
        Dx is interrupted if no response is received. Default is 5.
        seconds.

    Attributes
    ----------
    dx_type : str
        The type of the Dx.
    name : str
        The name of the Dx.
    port : str
        The port to which the Dx is connected.
    ser : Serial
        The serial device to communicate with the Dx.
    errors : dict
        Contains all error bits and their meanings.

        **Dictionary format :** {id (int): name (str)}
    dx_ids : list of int
        Contains the IDs of all Dx currently connected.

    Raises
    ------
    DynamixelError
        If the ID is wrong or is already registered, or if the Dynamixel with
        that ID is not an AX-12.

    """
    _max_turn_angle = 300

    _registers = ChainMap(
        {
            "down_calibration": 0x14,
            "up_calibration": 0x16,
            "cw_compliance_margin": 0x1A,
            "ccw_compliance_margin": 0x1B,
            "cw_compliance_slope": 0x1C,
            "ccw_compliance_slope": 0x1D
        },
        Dynamixel._registers
    )

    _register_maxima = ChainMap(
        {
            "cw_compliance_margin": 254,
            "ccw_compliance_margin": 254,
            "cw_compliance_slope": 254,
            "ccw_compliance_slope": 254
        },
        Dynamixel._register_maxima
    )

    def __init__(self, dx_id, name="AX-12",
                 port="/dev/ttyUSB0", baudrate=1000000, timeout=5):
        super().__init__(dx_id, dx_type="AX-12", name=name,
                         port=port, baudrate=baudrate, timeout=timeout)
        if self.model_number != 12:
            self.close()
            raise DynamixelError("Not an AX-12!")

    @property
    def down_calibration(self):
        """Read the down calibration register."""
        return self.read("down_calibration")

    @down_calibration.setter
    def down_calibration(self, value):
        """Write to the down calibration register."""
        self.write("down_calibration", value)

    @property
    def up_calibration(self):
        """Read the up calibration register."""
        return self.read("up_calibration")

    @up_calibration.setter
    def up_calibration(self, value):
        """Write to the up calibration register."""
        self.write("up_calibration", value)

    @property
    def cw_compliance_margin(self):
        """Read the clockwise compliance margin register."""
        return self.read("cw_compliance_margin")

    @cw_compliance_margin.setter
    def cw_compliance_margin(self, margin):
        """Write to the clockwise compliance margin register."""
        self.write("cw_compliance_margin", margin)

    @property
    def ccw_compliance_margin(self):
        """Read the counterclockwise compliance margin register."""
        return self.read("ccw_compliance_margin")

    @ccw_compliance_margin.setter
    def ccw_compliance_margin(self, margin):
        """Write to the counterclockwise compliance margin register."""
        self.write("ccw_compliance_margin", margin)

    @property
    def compliance_margins(self):
        """Return the clockwise and counterclockwise compliance margins."""
        return self.cw_compliance_margin, self.ccw_compliance_margin

    @compliance_margins.setter
    def compliance_margins(self, margins):
        """Set the clockwise and counterclockwise compliance margins."""
        try:
            cw_margin = margins[0]
            ccw_margin = margins[1]
        except TypeError:
            cw_margin = ccw_margin = margins

        if cw_margin is not None:
            self.cw_compliance_margin = cw_margin
        if ccw_margin is not None:
            self.ccw_compliance_margin = ccw_margin

    @property
    def cw_compliance_slope(self):
        """Read the clockwise compliance slope register."""
        return self.read("cw_compliance_slope")

    @cw_compliance_slope.setter
    def cw_compliance_slope(self, slope):
        """Write to the clockwise compliance slope register."""
        self.write("cw_compliance_slope", slope)

    @property
    def ccw_compliance_slope(self):
        """Read the counterclockwise compliance slope register."""
        return self.read("ccw_compliance_slope")

    @ccw_compliance_slope.setter
    def ccw_compliance_slope(self, slope):
        """Write to the counterclockwise compliance slope register."""
        self.write("ccw_compliance_slope", slope)

    @property
    def compliance_slopes(self):
        """Return the clockwise and counterclockwise compliance slopes."""
        return self.cw_compliance_slope, self.ccw_compliance_slope

    @compliance_slopes.setter
    def compliance_slopes(self, slopes):
        """Set the clockwise and counterclockwise compliance slopes."""
        try:
            cw_slope = slopes[0]
            ccw_slope = slopes[1]
        except TypeError:
            cw_slope = ccw_slope = slopes

        if cw_slope is not None:
            self.cw_compliance_slope = cw_slope
        if ccw_slope is not None:
            self.ccw_compliance_slope = ccw_slope


class MX28(Dynamixel):
    """
    The MX-28 series of Dx.

    Parameters
    ----------
    dx_id : int
        The current ID of the Dx.
    name : str, optional
        The name of the Dx. Default is "MX-28".
    port : str, optional
        The port to which the Dx is connected. Default is "/dev/ttyUSB0".
        Windows users can use COM ports instead.
    baudrate : int, optional
        The communication baudrate of the Dx. Default is 1,000,000.
    timeout : int, optional
        The length of time, in seconds, after which the communication with the
        Dx is interrupted if no response is received. Default is 5.
        seconds.

    Attributes
    ----------
    dx_type : str
        The type of the Dx.
    name : str
        The name of the Dx.
    port : str
        The port to which the Dx is connected.
    ser : Serial
        The serial device to communicate with the Dx.
    errors : dict
        Contains all error bits and their meanings.

        **Dictionary format :** {id (int): name (str)}
    dx_ids : list of int
        Contains the IDs of all Dx currently connected.

    Raises
    ------
    DynamixelError
        If the ID is wrong or is already registered, or if the Dynamixel with
        that ID is not an MX-28.

    """
    _max_turn_angle = 360

    _register_minima = Dynamixel._register_minima.copy()
    _register_maxima = Dynamixel._register_maxima.copy()

    _registers = ChainMap(
        {
            "multiturn_offset": 0x14,
            "resolution_divider": 0x16,
            "p_gain": 0x1A,
            "i_gain": 0x1B,
            "d_gain": 0x1C,
            "present_current": 0x38,
            "goal_acceleration": 0x49
        },
        Dynamixel._registers
    )

    _register_minima = ChainMap(
        {
            "multiturn_offset": -24576,
            "resolution_divider": 1
        },
        Dynamixel._register_minima
    )

    _register_maxima = ChainMap(
        {
            "multiturn_offset": 24576,
            "cw_limit": 4095,
            "ccw_limit": 4095,
            "resolution_divider": 4,
            "p_gain": 254,
            "i_gain": 254,
            "d_gain": 254,
            "goal_position": 4095,
            "goal_acceleration": 254
        },
        Dynamixel._register_maxima
    )

    _two_byte_registers = Dynamixel._two_byte_registers | {"multiturn_offset",
                                                           "present_current"}

    def __init__(self, dx_id, name="MX-28",
                 port="/dev/ttyUSB0", baudrate=1000000, timeout=5):
        super().__init__(dx_id, dx_type="MX-28", name=name,
                         port=port, baudrate=baudrate, timeout=timeout)
        if self.model_number != 29:
            self.close()
            raise DynamixelError("Not an MX-28!")

    @property
    def multiturn_offset(self):
        """Read the multiturn offset register."""
        return self.read("multiturn_offset")

    @multiturn_offset.setter
    def multiturn_offset(self, value):
        """Write to the multiturn offset register."""
        self.write("multiturn_offset", value)

    @property
    def resolution_divider(self):
        """Read the resolution divider register."""
        return self.read("resolution_divider")

    @resolution_divider.setter
    def resolution_divider(self, value):
        """Write to the resolution divider register."""
        self.write("resolution_divider", value)

    @property
    def p_gain(self):
        """Read the Kp register for PID gains."""
        return self.read("p_gain")

    @p_gain.setter
    def p_gain(self, value):
        """Write to the Kp register for PID gains."""
        self.write("p_gain", value)

    @property
    def i_gain(self):
        """Read the Ki register for PID gains."""
        return self.read("i_gain")

    @i_gain.setter
    def i_gain(self, value):
        """Write to the Ki register for PID gains."""
        self.write("i_gain", value)

    @property
    def d_gain(self):
        """Read the Kd register for PID gains."""
        return self.read("d_gain")

    @d_gain.setter
    def d_gain(self, value):
        """Write to the Kd register for PID gains."""
        self.write("d_gain", value)

    @property
    def current(self):
        """Get the current, in mA."""
        return self.read("present_current") * 10

    @property
    def goal_acceleration(self):
        """Read the goal acceleration register."""
        return self.read("goal_acceleration")

    @goal_acceleration.setter
    def goal_acceleration(self, value):
        """Write to the goal acceleration register."""
        self.write("goal_acceleration", value)

if __name__ == "__main__":
    dx = AX12(0)
    dx.led = False
