#!/usr/bin/env python

from serial import SerialException

import rospy

from ackermann_msgs.msg import AckermannDrive
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

from erp42_driver import ERP42Driver


class Node:
    def __init__(self, port_name):
        self.erp = ERP42Driver(port_name)

        rospy.Subscriber("/ackermann_cmd", AckermannDrive, self.callback)

        self.pub = rospy.Publisher("erp42_status", DiagnosticStatus, queue_size=1)

    def callback(self, msg):
        self.erp.set_vel(msg.speed)
        self.erp.set_steer(msg.steering_angle)
        self.erp.set_accel(msg.acceleration)

    def run(self):
        state = self.erp.sync()
        status = state._asdict()

        status["auto_mode"] = self.erp.get_auto_mode(state)
        status["e_stop"] = self.erp.get_e_stop(state)
        status["gear"] = self.erp.get_gear(state)
        status["speed"] = self.erp.get_vel(state, unit="km/h")
        status["steer"] = self.erp.get_steer(state, unit="deg")
        status["brake"] = state.brake
        status["alive"] = state.alive
        status["enc"] = state.enc

        values = []
        for key in status:
            values.append(KeyValue(key, str(status[key])))

        msg = DiagnosticStatus()
        msg.values = values

        self.pub.publish(msg)

        if state.valid is False:
            rospy.logwarn("Read failed: %s", state.raw)
        rospy.logdebug(state)

    def stop(self):
        self.erp.close()


if __name__ == "__main__":
    rospy.init_node("erp42_driver", log_level=rospy.DEBUG)
    rospy.loginfo("ERP42 driver Node")

    while not rospy.is_shutdown():
        port_name = rospy.get_param("~port", "/dev/ttyUSB0")
        rospy.loginfo("Connecting to %s" % (port_name))

        try:
            node = Node(port_name)
            break
        except SerialException as e:
            rospy.loginfo("Serial exception: %s" % (e))

        rospy.sleep(0.5)

    rospy.loginfo("Connected to %s" % (port_name))

    # node.run() blocks until ERP42 is ready
    while not rospy.is_shutdown():
        node.run()
