#!/usr/bin/env python

import time

import rospy

from ackermann_msgs.msg import AckermannDrive
from erp42_ros.msg import ERP42State

from erp42_driver.motor import ERP42


class Node:
    def __init__(self, port_name):
        self.erp = ERP42(port_name)

        self.pub = rospy.Publisher(
            "/erp42_state",
            ERP42State,
            queue_size=10,
        )
        self.sub = rospy.Subscriber(
            "/ackermann_cmd",
            AckermannDrive,
            self.callback,
            queue_size=10,
        )

    def callback(self, msg):
        self.erp.set_speed(msg.speed)
        self.erp.set_steer(msg.steering_angle)
        self.erp.set_accel(msg.acceleration)

    def run(self):
        state = self.erp.update()
        self.publish(state)

    def stop(self):
        self.erp.stop()
        time.sleep(0.2)
        self.erp.close()

    def publish(self, state):
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

        msg = ERP42State()
        msg.mode = mode_map[state.mode]
        msg.e_stop = e_stop_map[state.e_stop]
        msg.gear = gear_map[state.gear]

        msg.brake = state.brake
        msg.speed = float(state.speed) / 10
        msg.steer = float(state.steer) / 71
        msg.enc = state.enc
        msg.alive = state.alive

        self.pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("driver_node")
    rospy.loginfo("ERP42 driver Node")

    port_name = rospy.get_param("port", "/dev/ttyUSB0")
    rospy.loginfo("Connecting to %s" % (port_name))

    node = Node(port_name)
    rospy.loginfo("Connected to %s" % (port_name))
    while not rospy.is_shutdown():
        node.run()

    node.erp.close()
