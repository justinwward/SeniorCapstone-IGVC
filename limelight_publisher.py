#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float32, Int32, String

import limelight
import limelightresults


class LimelightPublisher(Node):
    def __init__(self):
        super().__init__('limelight_publisher')

        # Parameters
        self.declare_parameter('limelight_ip', '192.168.1.50')
        self.declare_parameter('pipe_stop', 0)
        self.declare_parameter('pipe_cone', 1)
        self.declare_parameter('pipe_lane', 2)
        self.declare_parameter('stop_min_area', 6.0)
        self.declare_parameter('cone_min_area', 1.0)
        self.declare_parameter('lane_min_area', 0.2)
        self.declare_parameter('pipeline_settle_time', 0.10)
        self.declare_parameter('stop_scan_interval', 0.20)
        self.declare_parameter('cone_scan_interval', 0.15)

        self.limelight_ip = self.get_parameter('limelight_ip').value
        self.PIPE_STOP = self.get_parameter('pipe_stop').value
        self.PIPE_CONE = self.get_parameter('pipe_cone').value
        self.PIPE_LANE = self.get_parameter('pipe_lane').value
        self.stop_min_area = self.get_parameter('stop_min_area').value
        self.cone_min_area = self.get_parameter('cone_min_area').value
        self.lane_min_area = self.get_parameter('lane_min_area').value
        self.pipeline_settle_time = self.get_parameter('pipeline_settle_time').value
        self.stop_scan_interval = self.get_parameter('stop_scan_interval').value
        self.cone_scan_interval = self.get_parameter('cone_scan_interval').value

        # Limelight connection
        self.ll = limelight.Limelight(self.limelight_ip)
        self.current_pipeline = None

        # Publishers
        self.stop_pub = self.create_publisher(Bool, '/limelight/stop_sign_detected', 10)
        self.cone_pub = self.create_publisher(Bool, '/limelight/cone_detected', 10)
        self.lane_visible_pub = self.create_publisher(Bool, '/limelight/lane_visible', 10)
        self.lane_error_pub = self.create_publisher(Float32, '/limelight/lane_error', 10)
        self.active_pipeline_pub = self.create_publisher(Int32, '/limelight/active_pipeline', 10)
        self.state_pub = self.create_publisher(String, '/limelight/state', 10)

        # Optional extra numeric outputs
        self.stop_tx_pub = self.create_publisher(Float32, '/limelight/stop_sign_tx', 10)
        self.stop_ta_pub = self.create_publisher(Float32, '/limelight/stop_sign_ta', 10)
        self.cone_tx_pub = self.create_publisher(Float32, '/limelight/cone_tx', 10)
        self.cone_ta_pub = self.create_publisher(Float32, '/limelight/cone_ta', 10)

        self.last_stop_scan = 0.0
        self.last_cone_scan = 0.0

        # Main loop timer
        self.timer = self.create_timer(0.05, self.loop)

        self.get_logger().info('Limelight perception node started')

    def switch_pipeline(self, index: int):
        if self.current_pipeline == index:
            return
        self.ll.pipeline_switch(index)
        self.current_pipeline = index
        time.sleep(self.pipeline_settle_time)

def get_results(self):
    try:
        raw = self.ll.get_results()
        parsed = limelightresults.parse_results(raw)
        return parsed
    except Exception as e:
        self.get_logger().warn(f'Failed to read Limelight: {e}')
        return None

    def publish_state(self, text: str):
        msg = String()
        msg.data = text
        self.state_pub.publish(msg)

    def publish_active_pipeline(self):
        msg = Int32()
        msg.data = int(self.current_pipeline) if self.current_pipeline is not None else -1
        self.active_pipeline_pub.publish(msg)

    def detector_match(self, results, keyword: str, min_area: float):
        if results is None:
            return False, 0.0, 0.0

        for d in results.detectorResults:
            label = str(d.className).lower()
            area = float(d.ta)
            tx = float(d.tx)

            if keyword in label and area >= min_area:
                return True, tx, area

        return False, 0.0, 0.0

    def lane_status(self, results):
        if results is None:
            return False, 0.0

        valid = bool(results.validity)
        tx = float(results.tx)
        ta = float(results.ta)

        if valid and ta >= self.lane_min_area:
            return True, tx

        return False, 0.0

    def loop(self):
        now = time.time()

        # ---- stop sign check ----
        if now - self.last_stop_scan >= self.stop_scan_interval:
            self.switch_pipeline(self.PIPE_STOP)
            self.publish_active_pipeline()

            results = self.get_results()
            found, tx, area = self.detector_match(results, 'stop', self.stop_min_area)

            self.stop_pub.publish(Bool(data=found))
            self.stop_tx_pub.publish(Float32(data=float(tx)))
            self.stop_ta_pub.publish(Float32(data=float(area)))

            if found:
                self.publish_state('STOP_SIGN_DETECTED')
            else:
                self.publish_state('STOP_SIGN_CLEAR')

            self.last_stop_scan = now

        # ---- cone check ----
        if now - self.last_cone_scan >= self.cone_scan_interval:
            self.switch_pipeline(self.PIPE_CONE)
            self.publish_active_pipeline()

            results = self.get_results()
            found, tx, area = self.detector_match(results, 'cone', self.cone_min_area)

            self.cone_pub.publish(Bool(data=found))
            self.cone_tx_pub.publish(Float32(data=float(tx)))
            self.cone_ta_pub.publish(Float32(data=float(area)))

            if found:
                self.publish_state('CONE_DETECTED')
            else:
                self.publish_state('CONE_CLEAR')

            self.last_cone_scan = now

        # ---- lane check ----
        self.switch_pipeline(self.PIPE_LANE)
        self.publish_active_pipeline()

        results = self.get_results()
        lane_visible, lane_error = self.lane_status(results)

        self.lane_visible_pub.publish(Bool(data=lane_visible))
        self.lane_error_pub.publish(Float32(data=float(lane_error)))

        if lane_visible:
            self.publish_state('LANE_VISIBLE')
        else:
            self.publish_state('LANE_LOST')


def main(args=None):
    rclpy.init(args=args)
    node = LimelightPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
