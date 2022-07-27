import math
import time

from erp42_driver.utils import ERP42Serial

MANUAL = 0
AUTO = 1

ESTOP_OFF = 0
ESTOP_ON = 1

FORWARD = 0
NEUTRAL = 1
REVERSE = 2


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


def to_kph(mps):
    return mps * 3.6


def to_mps(kph):
    return kph / 3.6


class ERP42Driver:
    """Driver for ERP42.

    Implements methods to set gear, speed, brake, steer with safety checks

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

        self.state = self.port.read()

        self.set_gear(NEUTRAL)
        self.set_vel(0.0)
        self.set_steer(0.0)
        self.set_accel(0.0)

    def get_auto_mode(self, state=None):
        """Gets current mode and converts to text

        Args:
            state (State): Optional state to convert

        Returns:
            (str): current Auto Mode as text
        """

        if state is None:
            state = self.state
        return mode_map[state.auto_mode]

    def get_e_stop(self, state=None):
        """Gets current E-Stop mode and converts to text

        Args:
            state (State): Optional state to convert

        Returns:
            (str): current E-Stop mode as text
        """

        if state is None:
            state = self.state
        return e_stop_map[state.e_stop]

    def get_gear(self, state=None):
        """Gets current gear as text

        Args:
            state (State): Optional state to convert

        Returns:
            (str): current gear as text
        """

        if state is None:
            state = self.state
        return gear_map[state.gear]

    def set_gear(self, gear):
        """Sets gear, mostly used internally

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

    def get_vel(self, state=None, unit="m/s"):
        """Gets current gear as text

        Args:
            state (State): Optional state to convert

        Returns:
            (str): current gear as text
        """

        if state is None:
            state = self.state

        gear = state.gear
        speed = state.speed

        if unit == "m/s":
            speed = to_mps(speed)
        elif unit != "km/h":
            raise NotImplementedError

        if gear == REVERSE:
            return -speed
        else:
            return speed

    def set_vel(self, vel, unit="km/h"):
        """Sets vel in meters per second

        Sets internal speed and gear value according to the ERP42 protocol o
        which accepts only positive speed values coupled with gear value.

        Args:
            vel (float): vel in m/s
        """

        if unit == "m/s":
            vel = to_kph(vel)
        elif unit != "km/h":
            raise NotImplementedError

        # convert velocity to speed and gear
        speed = vel
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

    def get_steer(self, state=None, unit="rad"):
        """Get steering angle in radians

        Args:
            unit (str): unit to return steering angle in, "rad" or "deg"

        Returns:
            (float): steering angle in radians

        Raises:
            NotImplementedError: if unit is undefined
        """

        if state is None:
            state = self.state

        degrees = state.steer
        if unit == "rad":
            return math.radians(degrees)
        elif unit == "deg":
            return degrees
        else:
            raise NotImplementedError

    def set_steer(self, angle, unit="rad"):
        """Set steering angle in radians

        Converts steering angle to ERP42 protocol format: angles * 71

        Args:
            angle (float):
                steering angle in radians, right is positive, left is negative
        """

        if unit == "rad":
            self.steer = math.degrees(angle)
        elif unit == "deg":
            self.steer = angle
        else:
            raise NotImplementedError

    def get_accel(self):
        raise NotImplementedError

    def set_accel(self, accel):
        """Adjust brake percentage according to acceleration

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
        """Reads and writes to ERP42Serial

        Synchronize class state with ERP42 state by reading states and sending
        command using internal state via the serial port

        This updates ERP42 state maintained internally by self.state,
        updates ERP42's physical state by sending internal desired state
        """

        data = self.port.read()
        if data.valid:
            self.state = data

        self.port.write(
            self.gear,
            self.speed,
            self.steer,
            self.brake,
            self.state.alive,
        )

        return self.state

    def close(self):
        """Closes connection to ERP42

        Stops the vehicle to prevent accidents wait for a full cylcle (20ms)
        for the command to propagate then close the serial port
        """

        self.stop()
        time.sleep(0.2)
        self.port.close()
