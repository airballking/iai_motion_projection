#!/usr/bin/env python

# TODO: add license header

import rospy
from iai_naive_kinematics_sim.msg import ProjectionClock
from iai_naive_kinematics_sim.srv import SetJointState
from iai_motion_projection.srv import TriggerProjection
from std_msgs.msg import Header

# TODO: document the class


class ProjectionManager:
    def __init__(self):
        rospy.init_node('projection_manager')
        self.iteration_finished = False
        self.last_clock_msg = ProjectionClock()
        self.clock_pub = rospy.Publisher('~projection_clock', ProjectionClock, queue_size=10, tcp_nodelay=True)
        self.ack_sub = rospy.Subscriber('~iteration_completed', Header, self.iteration_callback, queue_size=10, tcp_nodelay=True)
        self.set_joint_state = rospy.ServiceProxy('~set_joint_states', SetJointState, persistent=True)
        self.set_joint_state.wait_for_service(1.0)
        self.trigger_server = rospy.Service('~trigger_projection', TriggerProjection, self.projection_callback)
        # TODO: subscribe to topic with collision states

    def projection_callback(self, req):
        rospy.loginfo('received a trigger to start a projection')
        self.set_joint_state(req.state)
        self.last_clock_msg = ProjectionClock(rospy.Time(0.0), req.period)
        self.iteration_finished = False
        for i in range(req.iterations):
            self.clock_pub.publish(self.last_clock_msg)

            r = rospy.Rate(20000)
            while (not self.iteration_finished):
                r.sleep()

            self.iteration_finished = False
            self.last_clock_msg.now = self.last_clock_msg.now + self.last_clock_msg.period  # improve
        # TODO: fill response?
        return True

    def iteration_callback(self, data):
        if (self.last_clock_msg.now == data.stamp):
            self.iteration_finished = True
        else:
            self.iteration_finished = False
            rospy.logerr("Received an invalid message over topic '~iteration_completed'.")


# TODO: document main


def main():
    ProjectionManager()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSException as exc:
        rospy.loginfo("Died on an exception: " + str(exc))
