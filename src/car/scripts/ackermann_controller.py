#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class AckermannController:
    def __init__(self):
        rospy.init_node("ackermann_controller")

        # 車輛幾何參數
        self.wheelbase = rospy.get_param("~wheelbase", 0.23529)
        self.front_track = rospy.get_param("~front_track", 0.13)
        self.rear_track = rospy.get_param("~rear_track", 0.1685)
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.0325)

        # 限制
        self.max_steering = rospy.get_param("~max_steering", 0.6)
        self.max_wheel_speed = rospy.get_param("~max_wheel_speed", 30.0)

        # 控制更新頻率及逾時
        self.control_rate = rospy.get_param("~control_rate", 30.0)
        self.cmd_timeout = rospy.get_param("~cmd_timeout", 0.5)

        # Topic 可透過 launch 修改
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")

        self.left_steer_topic = rospy.get_param(
            "~left_steer_topic",
            "/front_left_steer_controller/command",
        )

        self.right_steer_topic = rospy.get_param(
            "~right_steer_topic",
            "/front_right_steer_controller/command",
        )

        self.left_wheel_topic = rospy.get_param(
            "~left_wheel_topic",
            "/back_left_wheel_controller/command",
        )

        self.right_wheel_topic = rospy.get_param(
            "~right_wheel_topic",
            "/back_right_wheel_controller/command",
        )

        self.left_steer_pub = rospy.Publisher(
            self.left_steer_topic,
            Float64,
            queue_size=10,
        )

        self.right_steer_pub = rospy.Publisher(
            self.right_steer_topic,
            Float64,
            queue_size=10,
        )

        self.left_wheel_pub = rospy.Publisher(
            self.left_wheel_topic,
            Float64,
            queue_size=10,
        )

        self.right_wheel_pub = rospy.Publisher(
            self.right_wheel_topic,
            Float64,
            queue_size=10,
        )

        # 保存最新 cmd_vel
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.last_cmd_time = rospy.Time(0)

        rospy.Subscriber(
            self.cmd_vel_topic,
            Twist,
            self.cmd_callback,
            queue_size=1,
        )

        # 固定頻率輸出關節命令
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.control_rate),
            self.control_loop,
        )

        rospy.on_shutdown(self.stop)

        rospy.loginfo("Ackermann controller started")
        rospy.loginfo("Listening cmd_vel topic: %s", self.cmd_vel_topic)
        rospy.loginfo(
            "wheelbase=%.5f, wheel_radius=%.5f",
            self.wheelbase,
            self.wheel_radius,
        )

    @staticmethod
    def clamp(value, minimum, maximum):
        return max(minimum, min(maximum, value))

    def cmd_callback(self, msg):
        self.target_linear = msg.linear.x
        self.target_angular = msg.angular.z
        self.last_cmd_time = rospy.Time.now()

        # 每 0.5 秒最多印一次，避免洗版
        rospy.loginfo_throttle(
            0.5,
            "Received /cmd_vel: linear.x=%.3f, angular.z=%.3f",
            self.target_linear,
            self.target_angular,
        )

    def control_loop(self, _event):
        # 沒收到 cmd_vel 或輸入逾時就停止
        if self.last_cmd_time == rospy.Time(0):
            self.publish_commands(0.0, 0.0, 0.0, 0.0)
            return

        elapsed = (rospy.Time.now() - self.last_cmd_time).to_sec()

        if elapsed > self.cmd_timeout:
            self.publish_commands(0.0, 0.0, 0.0, 0.0)
            return

        self.calculate_and_publish(
            self.target_linear,
            self.target_angular,
        )

    def calculate_and_publish(self, linear_velocity, angular_velocity):
        # 完全停止
        if (
            abs(linear_velocity) < 0.001
            and abs(angular_velocity) < 0.001
        ):
            self.publish_commands(0.0, 0.0, 0.0, 0.0)
            return

        # 阿克曼車無法原地旋轉
        # 速度為 0 時只轉動前輪，但後輪不驅動
        if abs(linear_velocity) < 0.001:
            steering = self.clamp(
                angular_velocity,
                -self.max_steering,
                self.max_steering,
            )

            left_steering, right_steering = (
                self.calculate_steering_angles(steering)
            )

            self.publish_commands(
                left_steering,
                right_steering,
                0.0,
                0.0,
            )
            return

        # cmd_vel angular.z 是車身目標角速度
        # delta = atan(L * omega / velocity)
        center_steering = math.atan(
            self.wheelbase * angular_velocity / linear_velocity
        )

        center_steering = self.clamp(
            center_steering,
            -self.max_steering,
            self.max_steering,
        )

        left_steering, right_steering = (
            self.calculate_steering_angles(center_steering)
        )

        # 左右後輪差速
        left_linear_velocity = (
            linear_velocity
            - angular_velocity * self.rear_track * 0.5
        )

        right_linear_velocity = (
            linear_velocity
            + angular_velocity * self.rear_track * 0.5
        )

        left_wheel_speed = (
            left_linear_velocity / self.wheel_radius
        )

        right_wheel_speed = (
            right_linear_velocity / self.wheel_radius
        )

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

        rospy.loginfo_throttle(
            0.5,
            (
                "Output: steer L=%.3f R=%.3f, "
                "wheel L=%.3f R=%.3f"
            ),
            left_steering,
            right_steering,
            left_wheel_speed,
            right_wheel_speed,
        )

        self.publish_commands(
            left_steering,
            right_steering,
            left_wheel_speed,
            right_wheel_speed,
        )

    def calculate_steering_angles(self, center_steering):
        if abs(center_steering) < 0.0001:
            return 0.0, 0.0

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
            left_denominator = math.copysign(
                0.001,
                left_denominator,
            )

        if abs(right_denominator) < 0.001:
            right_denominator = math.copysign(
                0.001,
                right_denominator,
            )

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

        return left_steering, right_steering

    def publish_commands(
        self,
        left_steering,
        right_steering,
        left_wheel_speed,
        right_wheel_speed,
    ):
        self.left_steer_pub.publish(
            Float64(data=left_steering)
        )

        self.right_steer_pub.publish(
            Float64(data=right_steering)
        )

        self.left_wheel_pub.publish(
            Float64(data=left_wheel_speed)
        )

        self.right_wheel_pub.publish(
            Float64(data=right_wheel_speed)
        )

    def stop(self):
        self.publish_commands(0.0, 0.0, 0.0, 0.0)
        rospy.loginfo("Ackermann controller stopped")


if __name__ == "__main__":
    try:
        controller = AckermannController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass