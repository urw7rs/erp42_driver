#!/usr/bin/env python

import struct
import logging

from serial import Serial
from collections import namedtuple

State = namedtuple(
    "State", "auto_mode e_stop gear speed steer brake enc alive raw valid"
)


class ERP42Serial:
    """Class representing serial port of ERP42

    Args:
        port_name (str): serial port name
        baud (int): serial port baudrate, default is 115200 according to ERP42 docs

    """

    def __init__(self, port_name, baud):
        self.port = Serial(port_name, baudrate=baud, timeout=1.0, write_timeout=1.0)

    def read(self):
        """Reads one data packet from ERP42 via uart

        Returns:
            (State): State tuple containing auto mode, e_stop, gear, speed, steer,
                brake, enc, alive values with raw line and valid flag. Sets valid flag
                to false if read packet isn't valid
        """

        line = self.port.readline()

        if len(line) != 18:
            valid = False

            line_as_hex = ":".join("{:02x}".format(ord(c)) for c in line)
            logging.warning("Read packet is invalid, skipping read: %s", line_as_hex)
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
            speed=float(speed) / 10,  # speed = actual speed (KPH) * 10
            steer=steer / 71,  # steer = actual steering dgree (dgree) * 71
            brake=brake,
            enc=enc,
            alive=alive,
            raw=line,
            valid=valid,
        )

    def write(self, gear, speed, steer, brake, alive):
        """Writes gear, speed, steer, brake, alive value to ERP42

        Creates a command packet from arguments and sends it via uart

        Args:
            gear (int): one of 0, 1, 2. 0 is forward, 1 is netural, 2 is reverse
            speed (int): speed in kph multiplied by 10 range is 0 to 200.
            steer (int): steering angle in degrees multiplied by 71
                range is -2000 to 2000
            brake (int): brake value without units 1 is no brakes 200 is full brakes
            alive (int): incremented to keep connection alive range is 0 to 255
        """

        if alive < 255:
            alive += 1
        else:
            alive = 0

        speed = int(speed * 10)  # actual speed (KPH) * 10
        steer = -int(steer * 71)  # negative of actual steering dgree (dgree) * 71

        assert gear in [0, 1, 2]
        assert speed >= 0 and speed <= 200
        assert steer >= -2000 and steer <= 2000
        assert brake >= 0 and brake <= 200
        assert alive >= 0 and alive <= 255

        self.port.write(
            struct.pack(
                ">BBBBBBhhBBBB",
                0x53,
                0x54,
                0x58,
                0x01,  # Auto Mode
                0x00,  # E-Stop Off
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
        """Closes serial port"""

        self.port.close()
