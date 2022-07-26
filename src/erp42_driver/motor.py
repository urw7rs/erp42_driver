import math
import time
import struct
import logging

from serial import Serial
from collections import namedtuple

MANUAL = 0
AUTO = 1

ESTOP_OFF = 0
ESTOP_ON = 1

FORWARD = 0
NEUTRAL = 1
REVERSE = 2


State = namedtuple(
    "State", "auto_mode e_stop gear speed steer brake enc alive raw valid"
)


class ERP42Serial:
    """
    class representing serial port of ERP42

    Args:
        port_name (str): serial port name
        baud (int): serial port baudrate, default is 115200 according to ERP42 docs

    """

    def __init__(self, port_name, baud):
        self.port = Serial(port_name, baudrate=baud)

    def read(self):
        """
        Read single data packet from ERP42 via uart

        Returns:
            (State): State tuple containing auto mode, e_stop, gear,
                     speed, steer, brake, enc, alive values with
                     raw line and valid flag

                     Sets valid flag to false if read packet isn't valid
        """

        line = self.port.readline()

        # return None if length doesn't match
        if len(line) != 18:
            logging.warning("Read packet is invalid, skipping read: %s", line)
            valid = False

            data = [0] * 8
        else:
            valid = True

            data = struct.unpack(
                "<BBBhhBiB",
                line[3:16],
            )

        auto_mode, e_stop, gear, speed, steer, brake, enc, alive = data

        return State(
            auto_mode=auto_mode,
            e_stop=e_stop,
            gear=gear,
            speed=speed,
            steer=steer,
            brake=brake,
            enc=enc,
            alive=alive,
            raw=line,
            valid=valid,
        )

    def write(self, gear, speed, steer, brake, alive):
        """
        Writes gear, speed, steer, brake, alive value to ERP42

        Creates a command packet from arguments and sends it via uart
        """

        if alive < 255:
            alive += 1
        else:
            alive = 0

        self.port.write(
            struct.pack(
                ">BBBBBBhhBBBB",
                0x53,
                0x54,
                0x58,
                AUTO,
                ESTOP_OFF,
                gear,
                speed,
                steer,
                brake,
                alive,
                0x0D,
                0x0A,
            )
        )

    def close(self):
        """
        Closes serial port
        """

        self.port.close()


mode_map = {
    0: "manual mode",
    1: "auto mode",
}
e_stop_map = {
    0: "E-STOP Off",
    1: "E-STOP On",
}
gear_map = {
    0: "forward drive",
    1: "neutral",
    2: "backward drive",
}


class ERP42Driver:
    """
    Driver for ERP42, implements methods to set
    gear, speed, brake, steer with safety checks

    Tracks ERP42 state in self.state and tracks desired state in
    self.gear, self.speed, self.brake, self.steer

    Running self.sync synchronizes interal state with physical ERP42 state
    by reading ERP42 state from serial port to self.state and sending internal
    desired state as commands to ERP42

    Args:
        port_name (str): serial port path
    """

    def __init__(self, port_name):
        self.port = ERP42Serial(port_name, 115200)

        self.state = State(0, 0, 0, 0, 0, 0, 0, 0, "", False)

        self.gear = 0
        self.speed = 0
        self.brake = 1
        self.steer = 0

        self.set_gear(NEUTRAL)
        self.set_speed(0.0)
        self.set_steer(0.0)
        self.set_accel(0.0)

    def get_auto_mode(self, state=None):
        """
        Gets current mode and converts to text

        Args:
            state (State): Optional state to convert

        Returns:
            (str): current Auto Mode as text
        """

        if state is None:
            state = self.state
        return mode_map[state.auto_mode]

    def get_e_stop(self, state=None):
        """
        Gets current E-Stop mode and converts to text

        Args:
            state (State): Optional state to convert

        Returns:
            (str): current E-Stop mode as text
        """

        if state is None:
            state = self.state
        return e_stop_map[state.e_stop]

    def get_gear(self, state=None):
        """
        Gets current gear as text

        Args:
            state (State): Optional state to convert

        Returns:
            (str): current gear as text
        """

        if state is None:
            state = self.state
        return gear_map[state.gear]

    def get_raw_gear(self):
        """
        Gets current raw gear value

        Returns:
            (int): gear value defined in the protocol
        """

        return self.state.gear

    def set_gear(self, gear):
        """
        Sets gear, mostly used internally

        Prevents users from changing gears in the opposite direction,
        e.g. changing to reverse when going forwards.

        If gear change is invalid, sets speed to state speed value.

        Args:
            gear (int): 0 for FORWARD, 1 for NEUTRAL, 2 for REVERSE
        """

        if (
            self.state.enc > 0
            and gear == REVERSE
            or self.state.enc < 0
            and gear == FORWARD
        ):
            self.speed = self.state.speed
            self.gear = self.state.gear
        else:
            self.gear = gear

    def get_speed(self, speed):
        gear = self.state.gear
        speed = self.state.speed
        if gear == REVERSE:
            return -speed
        else:
            return speed

    def set_speed(self, speed):
        """
        Sets speed in meters per second

        Sets internal speed and gear value according to the ERP42 protocol o
        which accepts only positive speed values coupled with gear value.

        Args:
            speed (float): speed in m/s
        """

        # convert to km/h
        speed = speed * 3.6 * 10

        # remove direction in speed and put it in gear value
        if speed < 0:
            speed = -speed
            gear = REVERSE

        if speed > 0:
            gear = FORWARD

        if speed == 0:
            gear = NEUTRAL

        # set speed after gear so set_gear overwrites speed if gear is invalid
        self.speed = speed
        self.set_gear(gear)

    def get_radians_steer(self):
        """
        Get steering angle in radians
        Returns:
            (float): steering angle in radians
        """

        return math.radians(self.state.steer / 71)

    def get_degrees_steer(self):
        """
        Get steering angle in radians
        Returns:
            (float): steering angle in radians
        """

        return math.radians(self.state.steer / 71)

    def set_steer(self, angle):
        """
        Set steering angle in radians

        Converts steering angle to ERP42 protocol format: angles * 71

        Args:
            angle (float):
                steering angle in radians, right is positive, left is negative
        """

        self.steer = math.degrees(angle) * 71

    def get_accel(self):
        raise NotImplementedError

    def set_accel(self, accel):
        """
        Adjust brake percentage according to acceleration

        Brake values aren't calculated to match acceleration

        Args:
            accel (float):
                acceleration in m/s^2, currently only accepts -1.0 to 0.0,
                -1.0 being 100% brake pressure, 0.0 0% brake pressure
        """

        if accel < 0:
            brake = accel * -200
        else:
            # ERP42 protocol docs says brake value of 1 is 0% brake pressure
            brake = 1

        self.brake = brake

    def stop(self):
        """
        Stop vehicle by applying full brakes
        """

        self.set_accel(-1.0)

    def get_state(self):
        """
        Get the state of the physical ERP42

        Returns:
            (State): State tuple containing auto mode, e_stop, gear,
                     speed, steer, brake, enc, alive values with
                     raw line and valid flag
        """

        return self.state

    def sync(self):
        """
        Synchronize class state with ERP42 state by reading states and sending
        command using internal state via the serial port

        This updates ERP42 state maintained internally by self.state,
        updates ERP42's physical state by sending internal desired state
        """

        data = self.port.read()
        if data.valid:
            self.state = data

        self.port.write(
            int(self.gear),
            int(self.speed),
            int(-self.steer),
            int(self.brake),
            int(self.state.alive),
        )

        return self.state

    def close(self):
        """
        Closes connection to ERP42

        Stops the vehicle to prevent accidents wait for a full cylcle (20ms)
        for the command to propagate then close the serial port
        """

        self.stop()
        time.sleep(0.2)
        self.port.close()
