#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class AckermannController:
    def __init__(self):
        rospy.init_node("ackermann_controller")

        # 從 URDF 關節位置估算
        # 前輪 x = 0.11922
        # 後輪 x = -0.11607
        self.wheelbase = rospy.get_param("~wheelbase", 0.23529)

        # 前輪轉向關節 y = ±0.065
        self.front_track = rospy.get_param("~front_track", 0.13)

        # 後輪關節 y = ±0.08425
        self.rear_track = rospy.get_param("~rear_track", 0.1685)

        # 請依照實際 STL 車輪尺寸調整
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.0325)

        # URDF 的 steering joint limit 是 ±0.6 rad
        self.max_steering = rospy.get_param("~max_steering", 0.6)
        self.max_wheel_speed = rospy.get_param("~max_wheel_speed", 30.0)

        self.left_steer_pub = rospy.Publisher(
            "/front_left_steer_controller/command",
            Float64,
            queue_size=10,
        )

        self.right_steer_pub = rospy.Publisher(
            "/front_right_steer_controller/command",
            Float64,
            queue_size=10,
        )

        self.left_wheel_pub = rospy.Publisher(
            "/back_left_wheel_controller/command",
            Float64,
            queue_size=10,
        )

        self.right_wheel_pub = rospy.Publisher(
            "/back_right_wheel_controller/command",
            Float64,
            queue_size=10,
        )

        rospy.Subscriber("/cmd_vel", Twist, self.cmd_callback)

        rospy.on_shutdown(self.stop)
        rospy.loginfo("Ackermann controller started, listening on /cmd_vel")

    @staticmethod
    def clamp(value, minimum, maximum):
        return max(minimum, min(maximum, value))

    def cmd_callback(self, msg):
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # 阿克曼車無法原地旋轉
        if abs(linear_velocity) < 0.001:
            self.publish_commands(0.0, 0.0, 0.0, 0.0)
            return

        # 單軌模型的中央轉向角
        center_steering = math.atan(
            self.wheelbase * angular_velocity / linear_velocity
        )

        center_steering = self.clamp(
            center_steering,
            -self.max_steering,
            self.max_steering,
        )

        steering_tangent = math.tan(center_steering)

        left_denominator = (
            self.wheelbase
            - self.front_track * 0.5 * steering_tangent
        )

        right_denominator = (
            self.wheelbase
            + self.front_track * 0.5 * steering_tangent
        )

        if abs(left_denominator) < 0.001:
            left_denominator = math.copysign(0.001, left_denominator)

        if abs(right_denominator) < 0.001:
            right_denominator = math.copysign(0.001, right_denominator)

        left_steering = math.atan(
            self.wheelbase
            * steering_tangent
            / left_denominator
        )

        right_steering = math.atan(
            self.wheelbase
            * steering_tangent
            / right_denominator
        )

        left_steering = self.clamp(
            left_steering,
            -self.max_steering,
            self.max_steering,
        )

        right_steering = self.clamp(
            right_steering,
            -self.max_steering,
            self.max_steering,
        )

        # 左轉時左後輪較慢、右後輪較快
        left_linear_velocity = (
            linear_velocity
            - angular_velocity * self.rear_track * 0.5
        )

        right_linear_velocity = (
            linear_velocity
            + angular_velocity * self.rear_track * 0.5
        )

        left_wheel_speed = left_linear_velocity / self.wheel_radius
        right_wheel_speed = right_linear_velocity / self.wheel_radius

        left_wheel_speed = self.clamp(
            left_wheel_speed,
            -self.max_wheel_speed,
            self.max_wheel_speed,
        )

        right_wheel_speed = self.clamp(
            right_wheel_speed,
            -self.max_wheel_speed,
            self.max_wheel_speed,
        )

        self.publish_commands(
            left_steering,
            right_steering,
            left_wheel_speed,
            right_wheel_speed,
        )

    def publish_commands(
        self,
        left_steering,
        right_steering,
        left_wheel_speed,
        right_wheel_speed,
    ):
        self.left_steer_pub.publish(Float64(data=left_steering))
        self.right_steer_pub.publish(Float64(data=right_steering))
        self.left_wheel_pub.publish(Float64(data=left_wheel_speed))
        self.right_wheel_pub.publish(Float64(data=right_wheel_speed))

    def stop(self):
        self.publish_commands(0.0, 0.0, 0.0, 0.0)


if __name__ == "__main__":
    try:
        AckermannController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass