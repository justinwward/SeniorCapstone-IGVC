
import math
from dataclasses import dataclass
from collections import defaultdict

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


@dataclass
class SectorStats:
    """
    Stores useful information about one steering sector.

    valid_points:
        Number of lidar points that passed all filters and landed in this sector.

    close_points:
        Number of valid points that are closer than stop_distance.

    nearest_distance:
        Closest valid point seen in this sector.

    occupied_cells:
        Number of grid cells that contain enough points to count as occupied.

    blocked:
        Whether this sector is considered blocked.
    """
    valid_points: int = 0
    close_points: int = 0
    nearest_distance: float = float("inf")
    occupied_cells: int = 0
    blocked: bool = False


class ObstacleDetectorNode(Node):
    """
    ROS 2 obstacle detector node.

    This node:
    - subscribes to the lidar point cloud
    - filters bad or irrelevant points
    - divides the forward area into steering sectors
    - scores those sectors based on obstacle density and distance
    - chooses the best direction
    - publishes Twist commands to /cmd_vel

    The motor driver is still responsible for converting /cmd_vel into wheel motion.
    """

    def __init__(self) -> None:
        """
        Initializes the ROS node, parameters, subscribers, publishers, and state.
        """
        super().__init__("obstacle_detector_node")

        # Subscribe to the lidar point cloud.
        # cloud_callback() runs every time a new scan is received.
        self.subscription = self.create_subscription(
            PointCloud2,
            "/cloud_unstructured_fullframe",
            self.cloud_callback,
            qos_profile_sensor_data,
        )

        # Publish movement commands.
        # The motor node subscribes to /cmd_vel.
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Normal forward driving speed.
        self.forward_speed = 0.12

        ## These are parameters which can be changed later as needed

        # Turning speeds.
        # Soft turns are used for mild steering corrections.
        # Hard turns are used when the best opening is farther off-center.
        self.soft_turn_speed = 0.30
        self.hard_turn_speed = 0.60

        # General point filtering.
        # Points outside these ranges are ignored before any obstacle logic runs.
        self.min_distance = 0.20
        self.max_distance = 2.00
        self.min_z = -0.45
        self.max_z = 0.80

        # Reaction distances.
        # The center sector looks farther ahead than the side sectors.
        self.center_max_distance = 1.00
        self.side_max_distance = 0.65

        # Used as an extra statistic for scoring and debugging.
        self.stop_distance = 0.70

        # Occupancy grid settings.
        # Nearby points are grouped into grid cells so isolated noise is ignored.
        self.cell_size = 0.20
        self.min_points_per_cell = 5

        # Steering sector angles.
        # The forward field is split into five sectors:
        # hard_left, left, center, right, hard_right.
        self.center_half_angle_deg = 18
        self.left_split_deg = 42
        self.outer_limit_deg = 75

        # Center corridor width.
        # This keeps the true forward driving path narrow so side clutter does not
        # falsely block forward motion.
        self.center_corridor_half_width = 0.30

        # Occupied-cell thresholds for declaring a sector blocked.
        self.center_min_cells_to_block = 3
        self.side_min_cells_to_block = 3

        # Clear thresholds for hysteresis.
        # These are lower than the block thresholds to prevent flickering.
        self.center_cells_to_clear = 1
        self.side_cells_to_clear = 1

        # Persistence settings.
        # A sector must be bad for multiple scans before blocking, and good for
        # multiple scans before clearing.
        self.block_confirm_scans = 2
        self.clear_confirm_scans = 4

        # Counters used by persistence logic.
        self.bad_counts = {
            "center": 0,
            "left": 0,
            "right": 0,
        }
        self.good_counts = {
            "center": 0,
            "left": 0,
            "right": 0,
        }

        # Persistent blocked states.
        # These store the filtered block state over time instead of relying on a
        # single lidar scan.
        self.blocked_states = {
            "center": False,
            "left": False,
            "right": False,
        }

        # Turn commitment.
        # Once a turn is selected, the robot keeps that direction briefly to avoid
        # oscillating left/right between scans.
        self.turn_lock_scans = 8
        self.turn_lock_remaining = 0
        self.turn_choice = "center"

        # Debug output timer.
        self.last_log_time = self.get_clock().now()

        self.get_logger().info("Obstacle detector READY (best-direction mode)")

    def point_valid(self, x, y, z):
        """
        Returns True if a lidar point should be used for obstacle detection.
        """
        dist = math.sqrt(x * x + y * y)

        if dist < self.min_distance:
            return False

        if dist > self.max_distance:
            return False

        if z < self.min_z or z > self.max_z:
            return False

        return True

    def get_steering_sector(self, x, y):
        """
        Converts a point into one of the five forward steering sectors.

        Points outside the forward steering region are labeled as back and ignored.
        """
        angle = math.degrees(math.atan2(y, x))

        if -self.center_half_angle_deg <= angle <= self.center_half_angle_deg:
            return "center"

        elif self.center_half_angle_deg < angle <= self.left_split_deg:
            return "left"

        elif self.left_split_deg < angle <= self.outer_limit_deg:
            return "hard_left"

        elif -self.left_split_deg <= angle < -self.center_half_angle_deg:
            return "right"

        elif -self.outer_limit_deg <= angle < -self.left_split_deg:
            return "hard_right"

        else:
            return "back"

    def point_in_reaction_range(self, sector, dist):
        """
        Returns True if the point is close enough to matter for its sector.
        """
        if sector == "center":
            return dist <= self.center_max_distance

        elif sector in ("left", "right", "hard_left", "hard_right"):
            return dist <= self.side_max_distance

        return False

    def get_cell(self, x, y):
        """
        Converts x-y coordinates into a grid cell index.
        """
        return (
            int(math.floor(x / self.cell_size)),
            int(math.floor(y / self.cell_size)),
        )

    def update_state(
        self,
        occupied,
        blocked_state,
        bad_count,
        good_count,
        block_threshold,
        clear_threshold,
    ):
        """
        Updates a persistent blocked state using hysteresis and scan persistence.
        """
        if not blocked_state:
            if occupied >= block_threshold:
                bad_count += 1
                good_count = 0

                if bad_count >= self.block_confirm_scans:
                    blocked_state = True
                    bad_count = 0
            else:
                bad_count = 0
                good_count += 1

        else:
            if occupied <= clear_threshold:
                good_count += 1
                bad_count = 0

                if good_count >= self.clear_confirm_scans:
                    blocked_state = False
                    good_count = 0
            else:
                good_count = 0
                bad_count += 1

        return blocked_state, bad_count, good_count

    def sector_score(self, stats: SectorStats):
        """
        Scores a steering sector.

        Higher score means the direction is more open.
        """
        nearest = stats.nearest_distance

        if nearest == float("inf"):
            nearest_term = 2.0
        else:
            nearest_term = nearest

        return (
            3.0 * nearest_term
            - 1.5 * stats.occupied_cells
            - 0.2 * stats.close_points
        )

    def choose_best_direction(self, sectors, left_blocked, right_blocked):
        """
        Chooses the best steering direction.

        If the center path is clear, the robot goes straight.
        If the center path is blocked, the robot scores available left/right sectors
        and chooses the most open option.
        """
        center_blocked = self.blocked_states["center"]

        if not center_blocked:
            return "center"

        if left_blocked and not right_blocked:
            candidates = ["right", "hard_right"]

        elif right_blocked and not left_blocked:
            candidates = ["left", "hard_left"]

        elif not left_blocked and not right_blocked:
            candidates = ["hard_left", "left", "right", "hard_right"]

        else:
            return "stop"

        best_name = None
        best_score = -float("inf")

        for name in candidates:
            score = self.sector_score(sectors[name])

            # Prefer softer turns slightly when scores are close.
            if name in ("left", "right"):
                score += 0.15

            if score > best_score:
                best_score = score
                best_name = name

        return best_name if best_name is not None else "stop"

    def steering_choice_to_angular(self, choice):
        """
        Converts a steering choice into angular velocity.
        """
        if choice == "center":
            return 0.0

        elif choice == "left":
            return self.soft_turn_speed

        elif choice == "hard_left":
            return self.hard_turn_speed

        elif choice == "right":
            return -self.soft_turn_speed

        elif choice == "hard_right":
            return -self.hard_turn_speed

        else:
            return 0.0

    def cloud_callback(self, msg: PointCloud2):
        """
        Main lidar callback.

        Processes the point cloud, updates obstacle states, chooses a steering
        direction, and publishes a Twist command.
        """
        sectors = {
            "hard_left": SectorStats(),
            "left": SectorStats(),
            "center": SectorStats(),
            "right": SectorStats(),
            "hard_right": SectorStats(),
        }

        cell_counts = {
            "hard_left": defaultdict(int),
            "left": defaultdict(int),
            "center": defaultdict(int),
            "right": defaultdict(int),
            "hard_right": defaultdict(int),
        }

        for x, y, z in point_cloud2.read_points(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True,
        ):
            if not self.point_valid(x, y, z):
                continue

            dist = math.sqrt(x * x + y * y)
            sector = self.get_steering_sector(x, y)

            # Ignore rear points completely.
            if sector == "back":
                continue

            # Keep the center sector limited to the true driving corridor.
            if sector == "center" and abs(y) > self.center_corridor_half_width:
                continue

            # Ignore points that are too far away to matter for the sector.
            if not self.point_in_reaction_range(sector, dist):
                continue

            stats = sectors[sector]
            stats.valid_points += 1

            if dist < stats.nearest_distance:
                stats.nearest_distance = dist

            if dist < self.stop_distance:
                stats.close_points += 1

            cell = self.get_cell(x, y)
            cell_counts[sector][cell] += 1

        # Count occupied grid cells in each sector.
        for sector in sectors:
            occupied = 0

            for count in cell_counts[sector].values():
                if count >= self.min_points_per_cell:
                    occupied += 1

            sectors[sector].occupied_cells = occupied

        # Combine soft and hard sectors into broader left/right block states.
        left_occupied = (
            sectors["left"].occupied_cells
            + sectors["hard_left"].occupied_cells
        )
        right_occupied = (
            sectors["right"].occupied_cells
            + sectors["hard_right"].occupied_cells
        )
        center_occupied = sectors["center"].occupied_cells

        self.blocked_states["center"], self.bad_counts["center"], self.good_counts["center"] = self.update_state(
            center_occupied,
            self.blocked_states["center"],
            self.bad_counts["center"],
            self.good_counts["center"],
            self.center_min_cells_to_block,
            self.center_cells_to_clear,
        )

        self.blocked_states["left"], self.bad_counts["left"], self.good_counts["left"] = self.update_state(
            left_occupied,
            self.blocked_states["left"],
            self.bad_counts["left"],
            self.good_counts["left"],
            self.side_min_cells_to_block,
            self.side_cells_to_clear,
        )

        self.blocked_states["right"], self.bad_counts["right"], self.good_counts["right"] = self.update_state(
            right_occupied,
            self.blocked_states["right"],
            self.bad_counts["right"],
            self.good_counts["right"],
            self.side_min_cells_to_block,
            self.side_cells_to_clear,
        )

        sectors["center"].blocked = self.blocked_states["center"]

        left_blocked = self.blocked_states["left"]
        right_blocked = self.blocked_states["right"]
        center_blocked = self.blocked_states["center"]

        cmd = Twist()

        if not center_blocked:
            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0

            self.turn_lock_remaining = 0
            self.turn_choice = "center"

        else:
            if self.turn_lock_remaining > 0 and self.turn_choice != "center":
                cmd.linear.x = 0.0
                cmd.angular.z = self.steering_choice_to_angular(self.turn_choice)
                self.turn_lock_remaining -= 1

            else:
                chosen = self.choose_best_direction(
                    sectors,
                    left_blocked,
                    right_blocked,
                )

                if chosen == "stop":
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0

                    self.turn_choice = "center"
                    self.turn_lock_remaining = 0

                else:
                    cmd.linear.x = 0.0
                    cmd.angular.z = self.steering_choice_to_angular(chosen)

                    if chosen == "center":
                        self.turn_choice = "center"
                        self.turn_lock_remaining = 0
                    else:
                        self.turn_choice = chosen
                        self.turn_lock_remaining = self.turn_lock_scans

        self.cmd_pub.publish(cmd)

        now = self.get_clock().now()

        if (now - self.last_log_time).nanoseconds > 1_000_000_000:
            self.get_logger().info(
                f"HL:{sectors['hard_left'].occupied_cells} "
                f"L:{sectors['left'].occupied_cells} "
                f"C:{sectors['center'].occupied_cells}({self.blocked_states['center']}) "
                f"R:{sectors['right'].occupied_cells} "
                f"HR:{sectors['hard_right'].occupied_cells} "
                f"| LB:{left_blocked} RB:{right_blocked} "
                f"| choice:{self.turn_choice} lock:{self.turn_lock_remaining} "
                f"| lin:{cmd.linear.x:.2f} ang:{cmd.angular.z:.2f}"
            )

            self.last_log_time = now


def main(args=None):
    """
    Standard ROS 2 entry point.
    """
    rclpy.init(args=args)

    node = ObstacleDetectorNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
