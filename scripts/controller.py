#!/usr/bin/env python3

import numpy as np
import rospy
from geometry_msgs.msg import Pose, Twist, Vector3
from sensor_msgs.msg import Joy
from tf.transformations import euler_from_quaternion


class Controller:
    def __init__(self) -> None:
        rospy.init_node("controller")

        self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback, queue_size=1)
        self.curr_pose_sub = rospy.Subscriber("curr_pose", Pose, self.curr_pose_callback, queue_size=1)

        # get frame name
        self.world_frame = str(rospy.get_param("world_frame", default="world"))
        self.agent_frame = str(rospy.get_param("agent_frame", default="agent"))

        self.timer_period = float(rospy.get_param("timer_period", default=0.01))
        self.timer = rospy.Timer(rospy.Duration(self.timer_period), self.timer_callback)

        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.cmd_vel_in_agent = Twist()
        self.cmd_vel_in_world = Twist()

    def joy_callback(self, msg: Joy) -> None:
        # invert value of x to match your vision
        self.cmd_vel_in_agent.linear = Vector3(x=-msg.axes[0], y=msg.axes[1])
        self.cmd_vel_in_agent.angular.z = msg.axes[3]

    def curr_pose_callback(self, msg: Pose) -> None:
        orientation = msg.orientation
        _, _, yaw = euler_from_quaternion(quaternion=[orientation.x, orientation.y, orientation.z, orientation.w])
        yaw += self.cmd_vel_in_agent.angular.z

        self.cmd_vel_in_world.linear = Vector3(
            x=self.cmd_vel_in_agent.linear.x * np.cos(yaw) - self.cmd_vel_in_agent.linear.y * np.sin(yaw),
            y=self.cmd_vel_in_agent.linear.x * np.sin(yaw) + self.cmd_vel_in_agent.linear.y * np.cos(yaw),
        )
        self.cmd_vel_in_world.angular.z = self.cmd_vel_in_agent.angular.z

    def timer_callback(self, timer: rospy.Timer) -> None:
        self.cmd_vel_pub.publish(self.cmd_vel_in_world)


def main() -> None:
    try:
        controller = Controller()
        rospy.spin()
    except Exception as e:
        rospy.logerr(f"{e}")


if __name__ == "__main__":
    main()
