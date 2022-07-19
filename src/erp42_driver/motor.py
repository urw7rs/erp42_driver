import time
import struct

from serial import Serial
from collections import namedtuple

MANUAL = 0
AUTO = 1

ESTOP_OFF = 0
ESTOP_ON = 1

FORWARD = 0
NEUTRAL = 1
REVERSE = 2

State = namedtuple("State", "mode e_stop gear speed steer brake enc alive")


class ERP42Serial:
    def __init__(self, port_name, baud):
        self.port = Serial(port_name, baudrate=baud, timeout=1.0)

    def read(self):
        line = self.port.readline()
        while len(line) != 18:
            line = self.port.readline()

        data = struct.unpack(
            "<BBBhhBiB",
            line[3:16],
        )

        return State(*data)

    def write(self, gear, speed, steer, brake, alive):
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
        self.port.close()


class ERP42:
    def __init__(self, port_name):
        self.port = ERP42Serial(port_name, 115200)

        self.state = State(0, 0, 0, 0, 0, 0, 0, 0)

        self.gear = 0
        self.speed = 0
        self.brake = 1
        self.steer = 0

        self.set_gear(NEUTRAL)
        self.set_speed(0.0)
        self.set_steer(0.0)
        self.set_accel(0.0)

    def set_gear(self, gear):
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

    def set_speed(self, speed):
        """set speed in kph"""
        speed *= 10

        if speed < 0:
            speed = -speed
            gear = REVERSE

        if speed > 0:
            gear = FORWARD

        self.set_gear(gear)
        self.speed = speed

    def set_steer(self, steer):
        """set steering angle in degrees"""
        self.steer = steer * 71

    def set_accel(self, accel):
        """adjust brake percentage according to accel"""
        if accel < 0:
            brake = accel * -200
        else:
            brake = 0

        self.brake = brake

    def stop(self):
        """stop vehicle by applying full brakes"""
        self.set_accel(-1.0)

    def update(self):
        self.state = self.port.read()

        self.port.write(
            int(self.gear),
            int(self.speed),
            int(self.steer),
            int(self.brake),
            int(self.state.alive),
        )

        return self.state

    def close(self):
        self.stop()
        time.sleep(0.2)
        self.port.close()
