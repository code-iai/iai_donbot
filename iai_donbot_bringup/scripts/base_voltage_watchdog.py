#!/usr/bin/env python

"""
base_voltage_watchdog.py: Node to connect and monitor voltage of the omnidrive bus.
Warning respectively error is send when the voltage is too low.
"""

__author__      = "Jeroen Schaefer"

from std_msgs.msg import String
import rospy
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
import csv
import os.path
import datetime
import time

timer = 0
warn_thresh = 48
err_thresh = 46
# pub = []  #type:rospy.Publisher
filename = "/home/jesch/battery_info/voltage.csv"
write_file = True

def diagnostic_cb(msg):
    """
    Callback for the topic "diagnostics_agg". Function checks the bus voltages of
    the omnidrive. A warning is send via the ros log utility when the voltage if
    between 46 and 47 V and an error if the voltage if lower than 46 V.
    :param msg: Message received from the topic
    :type msg: DiagnosticArray
    """

    voltages = []
    global timer
    global pub
    global err_thresh
    global warn_thresh

    # Get all four voltages of the buses
    for component in msg.status:  # type: DiagnosticStatus
        if component.name == "/Other/omnidrive: boxy_omni_v1":
            for key in component.values: #type:KeyValue
                if "bus voltage" in key.key:
                    if "fl" in key.key:
                        fl = float(key.value)
                    elif "fr" in  key.key:
                        fr = float(key.value)
                    elif "bl" in  key.key:
                        bl = float(key.value)
                    elif "br" in  key.key:
                        br = float(key.value)
            voltages= [fl, fr, bl, br]

    if len(voltages) > 0:

        # Check voltages and post to topic and ROS debug
        min_voltage = min(voltages)  # get lowest voltage
        if (min_voltage < warn_thresh) and (min_voltage > err_thresh):
            timer += 1
            if (timer >= 60):
                rospy.loginfo("Voltage low. Please charge the batteries soon. Voltage is down to " + str(min_voltage) + " V.")
                timer = 0
            pub.publish("Lowest voltage is " + str(min_voltage) + " V and therefore low.")
        if (min_voltage <= err_thresh):
            rospy.logerr("Voltage too low. Please charge the batteries. Voltage is down to " + str(min_voltage) + " V.")
            pub.publish("Lowest voltage is " + str(min_voltage) + " V and therefore critically low.")
        if (min_voltage >= warn_thresh):
            timer += 1
            if (timer >= 300):
                rospy.loginfo("Voltage of base is okay.")
                timer = 0
            pub.publish("Voltage is " + str(min_voltage) + " V and therefore okay.")
        

        if write_file:
            # ts = datetime.datetime.now().timestamp()
            now = datetime.datetime.now()
            ts = time.mktime(now.timetuple())

            with open(filename, "a") as csvfile:
                csvwriter = csv.writer(csvfile)
                csvwriter.writerow([ts] + voltages)
            csvfile.close()

def init():
    """
    Function to start the node and connect to the topic.
    """

    global pub
    rospy.init_node("battery_voltage_watchdog")
    rospy.loginfo("Connecting battery voltage watchdog to diagnostics.")

    rospy.Subscriber("/diagnostics_agg", DiagnosticArray, diagnostic_cb)
    pub = rospy.Publisher("/battery_status", String, queue_size=10)

    if write_file:
        # Write data to CVS
        if not os.path.isfile(filename):
            with open(filename, "w") as csvfile:
                csvwriter = csv.writer(csvfile)
                csvwriter.writerow(["Timestamp", "Bus Voltage FL", "Bus Voltage FR", "Bus Voltage BL", "Bus Voltage BR"])
            csvfile.close()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    init()
