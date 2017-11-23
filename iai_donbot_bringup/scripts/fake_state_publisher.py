#!/usr/bin/env python

import rospy
from control_msgs.msg._JointTrajectoryControllerState import JointTrajectoryControllerState
from sensor_msgs.msg._JointState import JointState


class FakeStateTopic(object):
    def __init__(self):
        self.sub = rospy.Subscriber('arm/joint_states', JointState, self.cb, queue_size=10)
        self.pub = rospy.Publisher('/fake_state', JointTrajectoryControllerState, queue_size=10)
        rospy.sleep(.2)
        rospy.loginfo('state faker running')

    def cb(self, data):
        jtcs = JointTrajectoryControllerState()
        jtcs.header = data.header
        jtcs.actual.positions = data.position
        jtcs.actual.velocities = data.velocity
        jtcs.actual.accelerations = data.effort
        jtcs.joint_names = data.name
        self.pub.publish(jtcs)


if __name__ == "__main__":
    rospy.init_node("fake_state_topic")
    a = FakeStateTopic()
    rospy.spin()
