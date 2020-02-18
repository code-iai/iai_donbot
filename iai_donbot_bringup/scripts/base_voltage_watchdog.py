#!/usr/bin/env python

"""
base_voltage_watchdog.py: Node to connect and monitor voltage of the omnidrive bus.
Warning respectively error is send when the voltage is too low.
"""

__author__      = "Jeroen Schaefer"

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

timer = 0

def diagnostic_cb(msg):
    """
    Callback for the topic "diagnostics_agg". Function checks the bus voltages of
    the omnidrive. A warning is send via the ros log utility when the voltage if
    between 46 and 47 V and an error if the voltage if lower than 46 V.
    :param msg: Message received from the topic
    :type msg: DiagnosticArray
    """
    global timer

    voltages = []

    for component in msg.status: #type:DiagnosticStatus
        if component.name == "/Other/omnidrive: boxy_omni_v1":
            for key in component.values: #type:KeyValue
                if "bus voltage" in key.key:
                    voltages.append(key.value)

    if len(voltages) > 0:
        if (min(voltages) < 47) and (min(voltages) > 46):
            timer += 1
            if timer > 60:
                rospy.loginfo("Voltage low. Please charge the batteries soon.")
                timer = 0
        elif min(voltages) <= 46:
            rospy.logerr("Voltage too low. Please charge the batteries.")


def init():
    """
    Function to start the node and connect to the topic.
    """
    rospy.init_node("battery_voltage_watchdog")
    rospy.loginfo("Connecting battery voltage watchdog to diagnostics.")

    rospy.Subscriber("/diagnostics_agg", DiagnosticArray, diagnostic_cb)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    init()