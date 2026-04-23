#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import lgpio


class MotorNode(Node):
    # All of this was mostly copied from Jose's code with a few exceptions like the soft and hard turn parameters
    def __init__(self):
        super().__init__('motor_node')

        # ---------------- GPIO CHIP ----------------
        self.gpio = lgpio.gpiochip_open(4)

        # ---------------- PWM PINS ----------------
        # M1_PWM = one motor
        # M2_PWM = the other motor
        self.M1_PWM = 18
        self.M2_PWM = 19

        lgpio.gpio_claim_output(self.gpio, self.M1_PWM)
        lgpio.gpio_claim_output(self.gpio, self.M2_PWM)

        # ---------------- PWM SETTINGS ----------------
        self.frequency = 1000
        self.stop_duty = 75

        # Forward values for the motors
        self.forward_motor1 = 88
        self.forward_motor2 = 89

        # Reverse values for the motors
        self.reverse_motor1 = 62
        self.reverse_motor2 = 61

        # ---------------- TURN SETTINGS ----------------
        # Soft turns ihave lower values so the robot will turn slower
        # Hard turns have higher values (and one wheel turns clockwise while the other turns counterclockwise) so the robot turns faster
        self.soft_left_motor1 = 86
        self.soft_left_motor2 = 64

        self.hard_left_motor1 = self.forward_motor1
        self.hard_left_motor2 = self.reverse_motor2

        self.soft_right_motor1 = 66
        self.soft_right_motor2 = 87

        self.hard_right_motor1 = self.reverse_motor1
        self.hard_right_motor2 = self.forward_motor2

        # ---------------- CMD MATCHING ----------------
        self.forward_cmd = 0.12
        self.soft_turn_cmd = 0.30
        self.hard_turn_cmd = 0.60

        self.cmd_tolerance = 0.05
        self.last_state = "STOP"

        self.stop_motors()

        # ---------------- ROS SUBSCRIBER ----------------
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        self.get_logger().info('Motor node started. Listening to /cmd_vel')

    # ---------------- HELPERS ----------------
    # Functions we call depending on what we want the robot to do
    def set_motor_pwm(self, m1_duty, m2_duty):
        lgpio.tx_pwm(self.gpio, self.M1_PWM, self.frequency, m1_duty)
        lgpio.tx_pwm(self.gpio, self.M2_PWM, self.frequency, m2_duty)

    def stop_motors(self):
        self.set_motor_pwm(self.stop_duty, self.stop_duty)

    def drive_forward(self):
        self.set_motor_pwm(self.forward_motor1, self.forward_motor2)

    def soft_left_turn(self):
        self.set_motor_pwm(self.soft_left_motor1, self.soft_left_motor2)

    def hard_left_turn(self):
        self.set_motor_pwm(self.hard_left_motor1, self.hard_left_motor2)

    def soft_right_turn(self):
        self.set_motor_pwm(self.soft_right_motor1, self.soft_right_motor2)

    def hard_right_turn(self):
        self.set_motor_pwm(self.hard_right_motor1, self.hard_right_motor2)

    def log_state_once(self, state):
        if state != self.last_state:
            self.get_logger().info(state)
            self.last_state = state

    def approx_equal(self, value, target):
        return abs(value - target) <= self.cmd_tolerance

    # ---------------- CMD CALLBACK ----------------
    # Here we get the command from the /cmd_vel topic and then call the helper functions depending on the linear_x and angular_z values
    def cmd_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Call stop the mototrs if both are equal to 0
        if self.approx_equal(linear_x, 0.0) and self.approx_equal(angular_z, 0.0):
            self.stop_motors()
            self.log_state_once('STOP')
            return
        # Call drive forward if inear is close to forward_cmd and angular is zero
        if self.approx_equal(linear_x, self.forward_cmd) and self.approx_equal(angular_z, 0.0):
            self.drive_forward()
            self.log_state_once('FORWARD')
            return
        
        # Call soft left if the linear is close to zero and the angular is close to soft_turn_cmd
        if self.approx_equal(linear_x, 0.0) and self.approx_equal(angular_z, self.soft_turn_cmd):
            self.soft_left_turn()
            self.log_state_once('SOFT LEFT')
            return
        # Call hard left if the linear is close to zero and the angular is close to hard_turn_cmd
        if self.approx_equal(linear_x, 0.0) and self.approx_equal(angular_z, self.hard_turn_cmd):
            self.hard_left_turn()
            self.log_state_once('HARD LEFT')
            return
        # Call soft right if the linear is close to zero and the angular is close to -soft_turn_cmd
        if self.approx_equal(linear_x, 0.0) and self.approx_equal(angular_z, -self.soft_turn_cmd):
            self.soft_right_turn()
            self.log_state_once('SOFT RIGHT')
            return
        # Call hard right if the linear is close to zero and the angular is close to -hard_turn_cmd
        if self.approx_equal(linear_x, 0.0) and self.approx_equal(angular_z, -self.hard_turn_cmd):
            self.hard_right_turn()
            self.log_state_once('HARD RIGHT')
            return

        self.stop_motors()
        self.log_state_once(
            f'STOP (unexpected cmd_vel: lin={linear_x:.2f}, ang={angular_z:.2f})'
        )

    # ---------------- CLEANUP ----------------
    # All of this below was copied from Joses' code
    def destroy_node(self):
        self.stop_motors()
        lgpio.gpiochip_close(self.gpio)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()