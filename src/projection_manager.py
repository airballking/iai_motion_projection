#!/usr/bin/env python

# TODO: add license header

import rospy
from iai_naive_kinematics_sim.msg import ProjectionClock
from iai_motion_projection.srv import TriggerProjection
from std_msgs.msg import Header

# TODO: document the class


class ProjectionManager:
    def __init__(self):
        rospy.init_node('projection_manager')
        self.iteration_finished = False
        self.last_clock_msg = ProjectionClock()  # TODO: switch to token
        self.clock_pub = rospy.Publisher('~projection_clock', ProjectionClock, queue_size=10, tcp_nodelay=True)
        self.server = rospy.Service('~trigger_projection', TriggerProjection, self.projection_callback)
        self.ack_sub = rospy.Subscriber('~iteration_completed', Header, self.iteration_callback, queue_size=10, tcp_nodelay=True)
        # TODO: subscribe to topic with collision states

    def projection_callback(self, req):
        rospy.loginfo('received a trigger to start a projection')
        # TODO: set initial joint state
        self.last_clock_msg = ProjectionClock(rospy.Time(0.0), req.period)
        self.iteration_finished = False
        for i in range(req.iterations):
            self.clock_pub.publish(self.last_clock_msg)

            r = rospy.Rate(100)
            while (not self.iteration_finished):
                r.sleep()

            self.iteration_finished = False
            self.last_clock_msg.now = self.last_clock_msg.now + self.last_clock_msg.period  # improve
        # TODO: fill response?
        return True

    def iteration_callback(self, data):
        rospy.loginfo("Got a message")
        # TODO: check for token instead
        if (self.last_clock_msg.now == data.stamp):
            self.iteration_finished = True
        else:
            self.iteration_finished = False
            # TODO: raise an error


# TODO: document main


def main():
    ProjectionManager()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
