#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Transform, TransformStamped, Twist, Vector3
from std_msgs.msg import ColorRGBA, Header
from tf2_ros.buffer import Buffer
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker


class AgentBody:
    def __init__(self) -> None:
        rospy.init_node("agent_body")

        # initial position
        x_init = float(rospy.get_param("x_init", default=0))
        y_init = float(rospy.get_param("y_init", default=0))
        z_init = float(rospy.get_param("z_init", default=0))

        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback, queue_size=1)
        self.curr_pose_pub = rospy.Publisher("curr_pose", Pose, queue_size=1)
        self.curr_pose = Pose(
            position=Point(x=x_init, y=y_init, z=z_init), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )

        rospy.loginfo(f"initial_position: {self.curr_pose}")

        # get frame name
        self.world_frame = str(rospy.get_param("/world_frame", default="world"))
        tf_prefix = rospy.get_namespace().lstrip("/")
        self.agent_frame = tf_prefix + str(rospy.get_param("/agent_frame", default="base_link"))
        rospy.loginfo(f"agent_frame: {self.agent_frame}")

        # tf2
        self.tf_buffer = Buffer()
        self.broadcaster = TransformBroadcaster()

        self.line_width = float(rospy.get_param("~line_width", default=0.01))
        # agent trajectory
        self.trajectory_marker_pub = rospy.Publisher("trajectory_marker", Marker, queue_size=1)
        self.trajectory_marker = Marker(
            header=Header(stamp=rospy.Time.now(), frame_id=self.world_frame),
            ns="trajectory",
            id=0,
            action=Marker.ADD,
            type=Marker.LINE_STRIP,
            pose=Pose(orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)),
            scale=Vector3(x=self.line_width),
            color=ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0),
        )
        self.sampling_time = float(rospy.get_param("~sampling_time", default=0.01))
        rospy.loginfo(f"sampling_time: {self.sampling_time}")

    def cmd_vel_callback(self, msg: Twist) -> None:
        position = self.curr_pose.position
        self.curr_pose.position = Point(
            x=position.x + self.sampling_time * msg.linear.x,
            y=position.y + self.sampling_time * msg.linear.y,
            z=position.z + self.sampling_time * msg.linear.z,
        )
        orientation = self.curr_pose.orientation
        _, _, yaw = euler_from_quaternion(quaternion=[orientation.x, orientation.y, orientation.z, orientation.w])

        orientation_array = quaternion_from_euler(ai=0, aj=0, ak=yaw + self.sampling_time * msg.angular.z)
        self.curr_pose.orientation = Quaternion(
            x=orientation_array[0],
            y=orientation_array[1],
            z=orientation_array[2],
            w=orientation_array[3],
        )
        self.curr_pose_pub.publish(self.curr_pose)

        transform_stamped = TransformStamped(
            header=Header(stamp=rospy.Time.now(), frame_id=self.world_frame),
            child_frame_id=self.agent_frame,
            transform=Transform(
                translation=Vector3(
                    x=self.curr_pose.position.x,
                    y=self.curr_pose.position.y,
                    z=self.curr_pose.position.z,
                ),
                rotation=self.curr_pose.orientation,
            ),
        )
        self.broadcaster.sendTransform(transform_stamped)

        self.trajectory_marker.header.stamp = rospy.Time.now()
        # buffer制限
        if len(self.trajectory_marker.points) > 100:
            self.trajectory_marker.points.pop(0)
        self.trajectory_marker.points.append(self.curr_pose.position)
        self.trajectory_marker_pub.publish(self.trajectory_marker)


def main() -> None:
    try:
        agent_body = AgentBody()
        rospy.spin()
    except Exception as e:
        rospy.logerr(f"{e}")


if __name__ == "__main__":
    main()
