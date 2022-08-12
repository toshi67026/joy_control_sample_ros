#!/usr/bin/env python3

import numpy as np
import rospy
from cbf_utils.example_field_cbf_optimizer import FieldCBFOptimizer
from coverage_control.utils import get_color_rgba
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from numpy.typing import NDArray
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker


class CBFOptimizer:
    def __init__(self) -> None:
        rospy.init_node("cbf_optimizer")

        self.optimizer = FieldCBFOptimizer()

        self.activate_cbf = bool(rospy.get_param("/activate_cbf", default=True))

        # cbf計算に用いるパラメータを取得
        cbf_param = rospy.get_param("/cbf")
        limit: NDArray = np.array(cbf_param["limit"])
        dim = len(limit)
        self.theta = float(cbf_param["theta"])
        self.p = float(cbf_param["p"])
        self.keep_inside = bool(cbf_param["keep_inside"])
        rospy.loginfo(f"limit: {limit}")
        rospy.loginfo(f"theta: {self.theta}")
        rospy.loginfo(f"p: {self.p}")
        rospy.loginfo(f"keep_inside: {self.keep_inside}")

        self.cent_field: NDArray = np.array([sum(limit[i]) / dim for i in range(dim)])
        self.width: NDArray = np.array([(limit[i][1] - limit[i][0]) / 2.0 for i in range(dim)])
        rospy.loginfo(f"cent_field: {self.cent_field}")
        rospy.loginfo(f"width: {self.width}")

        world_frame = str(rospy.get_param("/world_frame", default="world"))

        self.curr_pose = Pose()
        rospy.Subscriber("curr_pose", Pose, self.curr_pose_callback, queue_size=1)

        rospy.Subscriber("cmd_vel_nom", Twist, self.cmd_vel_nom_callback, queue_size=1)
        self.cmd_vel_opt_pub = rospy.Publisher("cmd_vel_opt", Twist, queue_size=1)

        orientation_array = quaternion_from_euler(ai=0, aj=0, ak=self.theta)
        # 障害物を表示
        self.obstacle_marker = Marker(
            header=Header(stamp=rospy.Time.now(), frame_id=world_frame),
            ns="obstacle_marker",
            action=Marker.ADD,
            type=Marker.CYLINDER,
            pose=Pose(
                position=Point(x=self.cent_field[0], y=self.cent_field[1], z=0.0),
                orientation=Quaternion(
                    x=orientation_array[0],
                    y=orientation_array[1],
                    z=orientation_array[2],
                    w=orientation_array[3],
                ),
            ),
            scale=Vector3(x=self.width[0] * 2, y=self.width[1] * 2, z=0.1),
            color=get_color_rgba(color_initial="w", alpha=0.5),
        )
        self.obstacle_marker_pub = rospy.Publisher("obstacle_marker", Marker, queue_size=1)

    def curr_pose_callback(self, msg: Pose) -> None:
        self.curr_pose = msg

    def cmd_vel_nom_callback(self, msg: Twist) -> None:
        cmd_vel_opt = msg
        if self.activate_cbf:
            self.optimizer.set_field_parameters(
                cent_field=self.cent_field, width=self.width, theta=self.theta, p=self.p, keep_inside=self.keep_inside
            )
            agent_position: NDArray = np.array([self.curr_pose.position.x, self.curr_pose.position.y])
            nominal_input: NDArray = np.array(
                [
                    msg.linear.x,
                    msg.linear.y,
                ]
            )
            # 設定した制約に基づく最適入力を計算
            _, optimal_input = self.optimizer.optimize(nominal_input, agent_position)
            cmd_vel_opt.linear = Vector3(x=float(optimal_input[0]), y=float(optimal_input[1]))

        self.cmd_vel_opt_pub.publish(cmd_vel_opt)

        self.obstacle_marker.header.stamp = rospy.Time.now()
        self.obstacle_marker_pub.publish(self.obstacle_marker)


def main() -> None:
    try:
        cbf_optimizer = CBFOptimizer()
        rospy.spin()
    except Exception as e:
        rospy.logerr(f"{e}")


if __name__ == "__main__":
    main()
