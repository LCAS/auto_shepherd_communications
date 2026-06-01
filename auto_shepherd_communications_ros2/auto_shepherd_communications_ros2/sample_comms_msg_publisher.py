#!/usr/bin/env python3

import json
import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from sensor_msgs.msg import Joy
from nav_msgs.msg import Path
from geographic_msgs.msg import GeoPoseStamped, GeoPath
from nav2_msgs.action import FollowWaypoints, NavigateToPose
from topological_navigation_msgs.action import GotoNode


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class ControlMessageSimulator(Node):
    def __init__(self):
        super().__init__("control_message_simulator")

        self.frame_id = "map"
        self.gps_frame_id = "wgs84"

        self.base_lat = 55.9533
        self.base_lon = -3.1883
        self.base_alt = 50.0

        self.path_radius = 15.0
        self.geo_step = 0.00005

        self.sequence_id = 0
        self.node_index = 0
        self.instruction_index = 0

        # Teleoperation
        self.twist_pub = self.create_publisher(Twist, "/sim/control/teleop/cmd_vel", 10)
        self.joy_pub = self.create_publisher(Joy, "/sim/control/teleop/joy", 10)

        # Path-based
        self.path_pub = self.create_publisher(Path, "/sim/control/path/path", 10)
        self.geo_path_pub = self.create_publisher(GeoPath, "/sim/control/path/geopath", 10)

        # Waypoint sequences
        self.follow_waypoints_pub = self.create_publisher(
            String, "/sim/control/waypoints/follow_waypoints_goal", 10
        )
        self.geo_waypoints_pub = self.create_publisher(
            GeoPath, "/sim/control/waypoints/geopath", 10
        )

        # Pose goal
        self.navigate_to_pose_pub = self.create_publisher(
            String, "/sim/control/pose_goal/navigate_to_pose_goal", 10
        )
        self.single_geo_goal_pub = self.create_publisher(
            GeoPoseStamped, "/sim/control/pose_goal/geopose", 10
        )

        # Graph goal
        self.graph_goal_pub = self.create_publisher(
            String, "/sim/control/graph_goal/goto_node_goal", 10
        )

        # Abstract instruction
        self.instruction_pub = self.create_publisher(
            String, "/sim/control/instruction/semantic_goal", 10
        )

        # Timers
        self.create_timer(0.05, self.publish_teleop)            # 20 Hz
        self.create_timer(5.0, self.publish_path_based)         # 0.2 Hz
        self.create_timer(10.0, self.publish_waypoint_sequence) # 0.1 Hz
        self.create_timer(2.0, self.publish_pose_goal)          # 0.5 Hz
        self.create_timer(5.0, self.publish_graph_goal)         # 0.2 Hz
        self.create_timer(20.0, self.publish_instruction)       # 0.05 Hz

    def make_pose_stamped(self, index: int, total: int) -> PoseStamped:
        t = self.get_clock().now().nanoseconds / 1e9
        theta = (2.0 * math.pi * index / total) + (0.1 * t)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = self.path_radius * math.cos(theta)
        msg.pose.position.y = self.path_radius * math.sin(theta)
        msg.pose.position.z = 0.0
        msg.pose.orientation = yaw_to_quaternion(theta + math.pi / 2.0)
        return msg

    def make_geo_pose_stamped(self, index: int, total: int) -> GeoPoseStamped:
        t = self.get_clock().now().nanoseconds / 1e9
        theta = (2.0 * math.pi * index / total) + (0.03 * t)

        msg = GeoPoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.gps_frame_id
        msg.pose.position.latitude = self.base_lat + self.geo_step * math.sin(theta)
        msg.pose.position.longitude = self.base_lon + self.geo_step * math.cos(theta)
        msg.pose.position.altitude = self.base_alt
        msg.pose.orientation = yaw_to_quaternion(theta)
        return msg

    def publish_teleop(self):
        t = self.get_clock().now().nanoseconds / 1e9

        twist = Twist()
        twist.linear.x = 1.0 * math.sin(0.7 * t)
        twist.linear.y = 0.2 * math.sin(0.4 * t)
        twist.angular.z = 0.6 * math.cos(0.5 * t)
        self.twist_pub.publish(twist)

        joy = Joy()
        joy.header.stamp = self.get_clock().now().to_msg()
        joy.header.frame_id = "joystick"
        joy.axes = [
            float(math.sin(t)),
            float(math.cos(t)),
            float(math.sin(0.5 * t)),
            float(math.cos(0.5 * t)),
            0.0,
            0.0,
            1.0 if int(t) % 8 < 4 else -1.0,
            1.0 if int(t) % 10 < 5 else -1.0,
        ]
        joy.buttons = [
            1 if int(t * 2) % 11 == 0 else 0,
            1 if int(t * 3) % 17 == 0 else 0,
            1 if int(t * 5) % 23 == 0 else 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0
        ]
        self.joy_pub.publish(joy)

    def publish_path_based(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.frame_id
        path.poses = [self.make_pose_stamped(i, 80) for i in range(80)]
        self.path_pub.publish(path)

        geo_path = GeoPath()
        geo_path.header.stamp = self.get_clock().now().to_msg()
        geo_path.header.frame_id = self.gps_frame_id
        geo_path.poses = [self.make_geo_pose_stamped(i, 80) for i in range(80)]
        self.geo_path_pub.publish(geo_path)

    def publish_waypoint_sequence(self):
        # Real waypoint list as GPS points
        geo_path = GeoPath()
        geo_path.header.stamp = self.get_clock().now().to_msg()
        geo_path.header.frame_id = self.gps_frame_id
        geo_path.poses = [self.make_geo_pose_stamped(i, 20) for i in range(20)]
        self.geo_waypoints_pub.publish(geo_path)

        # Realistic JSON equivalent of FollowWaypoints.Goal
        goal = FollowWaypoints.Goal()
        goal.poses = [self.make_pose_stamped(i, 20) for i in range(20)]

        msg = String()
        msg.data = json.dumps({
            "type": "FollowWaypoints.Goal",
            "sequence_id": self.sequence_id,
            "frame_id": self.frame_id,
            "waypoint_count": len(goal.poses),
            "poses": [
                {
                    "x": p.pose.position.x,
                    "y": p.pose.position.y,
                    "z": p.pose.position.z,
                    "qx": p.pose.orientation.x,
                    "qy": p.pose.orientation.y,
                    "qz": p.pose.orientation.z,
                    "qw": p.pose.orientation.w,
                }
                for p in goal.poses
            ]
        })
        self.follow_waypoints_pub.publish(msg)

    def publish_pose_goal(self):
        geo_goal = self.make_geo_pose_stamped(self.sequence_id % 100, 100)
        self.single_geo_goal_pub.publish(geo_goal)

        goal = NavigateToPose.Goal()
        goal.pose = self.make_pose_stamped(self.sequence_id % 40, 40)
        goal.behavior_tree = ""

        msg = String()
        msg.data = json.dumps({
            "type": "NavigateToPose.Goal",
            "sequence_id": self.sequence_id,
            "frame_id": goal.pose.header.frame_id,
            "pose": {
                "x": goal.pose.pose.position.x,
                "y": goal.pose.pose.position.y,
                "z": goal.pose.pose.position.z,
                "qx": goal.pose.pose.orientation.x,
                "qy": goal.pose.pose.orientation.y,
                "qz": goal.pose.pose.orientation.z,
                "qw": goal.pose.pose.orientation.w,
            },
            "behavior_tree": goal.behavior_tree
        })
        self.navigate_to_pose_pub.publish(msg)

    def publish_graph_goal(self):
        goal = GotoNode.Goal()

        node_name = f"node_{self.node_index:03d}"
        self.node_index = (self.node_index + 1) % 200

        # common field name for this action
        goal.target = node_name

        msg = String()
        msg.data = json.dumps({
            "type": "GotoNode.Goal",
            "sequence_id": self.sequence_id,
            "target": goal.target,
            "route_hint": ["corridor_a", "junction_2", goal.target],
            "planner_mode": "topological"
        })
        self.graph_goal_pub.publish(msg)

    def publish_instruction(self):
        instructions = [
            {
                "command": "go_to",
                "target": "loading_bay",
                "priority": "normal",
                "constraints": {"avoid_people": True, "max_speed": 1.2}
            },
            {
                "command": "inspect",
                "target": "row_4",
                "priority": "high",
                "constraints": {"capture_images": True, "pause_on_detection": True}
            },
            {
                "command": "return_to_base",
                "target": "charging_station",
                "priority": "normal",
                "constraints": {"dock": True}
            },
            {
                "command": "wait",
                "target": "gate_area",
                "priority": "low",
                "constraints": {"duration_s": 30}
            },
        ]

        instruction = instructions[self.instruction_index % len(instructions)]
        self.instruction_index += 1

        msg = String()
        msg.data = json.dumps({
            "type": "SemanticGoal",
            "sequence_id": self.sequence_id,
            **instruction
        })
        self.instruction_pub.publish(msg)

        self.sequence_id += 1


def main(args=None):
    rclpy.init(args=args)
    node = ControlMessageSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
