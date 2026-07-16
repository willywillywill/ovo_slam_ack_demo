#!/usr/bin/env python3

import math

import rospy
import tf

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion


class AckermannOdom:
    def __init__(self):
        rospy.init_node("ackermann_odom")

        # 車體參數，請依照你的 URDF 修改
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.0325)
        self.wheelbase = rospy.get_param("~wheelbase", 0.35)

        # Joint 名稱，請確認與 URDF 完全相同
        self.rear_left_joint = rospy.get_param(
            "~rear_left_joint",
            "back_left_joint"
        )

        self.rear_right_joint = rospy.get_param(
            "~rear_right_joint",
            "back_right_joint"
        )

        self.front_left_steer_joint = rospy.get_param(
            "~front_left_steer_joint",
            "front_left_steer_joint"
        )

        self.front_right_steer_joint = rospy.get_param(
            "~front_right_steer_joint",
            "front_right_steer_joint"
        )

        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.publish_tf = rospy.get_param("~publish_tf", True)

        # 車輛位置
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Joint 狀態
        self.rear_left_velocity = 0.0
        self.rear_right_velocity = 0.0

        self.front_left_angle = 0.0
        self.front_right_angle = 0.0

        self.received_joint_states = False

        self.last_time = rospy.Time.now()

        self.odom_pub = rospy.Publisher(
            "/odom",
            Odometry,
            queue_size=20
        )

        self.tf_broadcaster = tf.TransformBroadcaster()

        rospy.Subscriber(
            "/joint_states",
            JointState,
            self.joint_state_callback,
            queue_size=20
        )

        publish_rate = rospy.get_param("~publish_rate", 50.0)

        self.timer = rospy.Timer(
            rospy.Duration(1.0 / publish_rate),
            self.update
        )

        rospy.loginfo("Ackermann odom node started")

    def get_joint_index(self, msg, joint_name):
        try:
            return msg.name.index(joint_name)
        except ValueError:
            return None

    def joint_state_callback(self, msg):
        rear_left_index = self.get_joint_index(
            msg,
            self.rear_left_joint
        )

        rear_right_index = self.get_joint_index(
            msg,
            self.rear_right_joint
        )

        front_left_index = self.get_joint_index(
            msg,
            self.front_left_steer_joint
        )

        front_right_index = self.get_joint_index(
            msg,
            self.front_right_steer_joint
        )

        missing_joints = []

        if rear_left_index is None:
            missing_joints.append(self.rear_left_joint)

        if rear_right_index is None:
            missing_joints.append(self.rear_right_joint)

        if front_left_index is None:
            missing_joints.append(self.front_left_steer_joint)

        if front_right_index is None:
            missing_joints.append(self.front_right_steer_joint)

        if missing_joints:
            rospy.logwarn_throttle(
                5.0,
                "Missing joints in /joint_states: {}".format(
                    ", ".join(missing_joints)
                )
            )
            return

        try:
            self.rear_left_velocity = msg.velocity[rear_left_index]
            self.rear_right_velocity = msg.velocity[rear_right_index]

            self.front_left_angle = msg.position[front_left_index]
            self.front_right_angle = msg.position[front_right_index]

            self.received_joint_states = True

        except IndexError:
            rospy.logwarn_throttle(
                5.0,
                "/joint_states position or velocity array is incomplete"
            )

    def update(self, event):
        now = rospy.Time.now()

        dt = (now - self.last_time).to_sec()
        self.last_time = now

        if not self.received_joint_states:
            return

        if dt <= 0.0 or dt > 1.0:
            return

        # 左右後輪平均速度
        rear_wheel_angular_velocity = (
            self.rear_left_velocity +
            self.rear_right_velocity
        ) / 2.0

        linear_velocity = (
            rear_wheel_angular_velocity *
            self.wheel_radius
        )

        # 使用兩個前輪的平均轉向角
        steering_angle = (
            self.front_left_angle +
            self.front_right_angle
        ) / 2.0

        if abs(self.wheelbase) < 1e-6:
            rospy.logerr_throttle(
                5.0,
                "wheelbase must be greater than zero"
            )
            return

        angular_velocity = (
            linear_velocity *
            math.tan(steering_angle) /
            self.wheelbase
        )

        # 中點積分，比直接 Euler 積分稍準確
        yaw_mid = self.yaw + angular_velocity * dt * 0.5

        self.x += (
            linear_velocity *
            math.cos(yaw_mid) *
            dt
        )

        self.y += (
            linear_velocity *
            math.sin(yaw_mid) *
            dt
        )

        self.yaw += angular_velocity * dt

        # 將 yaw 限制在 -pi 到 pi
        self.yaw = math.atan2(
            math.sin(self.yaw),
            math.cos(self.yaw)
        )

        quaternion = tf.transformations.quaternion_from_euler(
            0.0,
            0.0,
            self.yaw
        )

        if self.publish_tf:
            self.tf_broadcaster.sendTransform(
                (self.x, self.y, 0.0),
                quaternion,
                now,
                self.base_frame,
                self.odom_frame
            )

        odom_msg = Odometry()

        odom_msg.header.stamp = now
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        odom_msg.pose.pose.orientation = Quaternion(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3]
        )

        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = angular_velocity

        # 簡單 covariance
        odom_msg.pose.covariance[0] = 0.05
        odom_msg.pose.covariance[7] = 0.05
        odom_msg.pose.covariance[35] = 0.10

        odom_msg.twist.covariance[0] = 0.05
        odom_msg.twist.covariance[7] = 0.05
        odom_msg.twist.covariance[35] = 0.10

        self.odom_pub.publish(odom_msg)


if __name__ == "__main__":
    try:
        AckermannOdom()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass