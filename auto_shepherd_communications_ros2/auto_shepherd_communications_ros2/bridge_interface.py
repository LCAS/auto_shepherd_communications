#!/usr/bin/env python3

import os
import json
import importlib
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from std_msgs.msg import String
from rosidl_runtime_py.convert import (
    message_to_ordereddict,
    populate_msg_from_dict,
)


class SerialBridge(Node):
    def __init__(self):
        super().__init__("serial_bridge")

        cfg = self.load_config()
        self.defaults = cfg.get("defaults", {})

        # String interface to “serial link”
        self.serial_in_pub = self.create_publisher(String, "/serial/in", 10)
        self.create_subscription(String, "/serial/out", self.serial_out_cb, 10)

        # Maps topic → publisher (for data arriving from serial)
        self.ros_publishers = {}
        # Keep msg classes handy
        self.msg_classes = {}

        for entry in cfg.get("topics", []):
            self.setup_topic(entry)

    # ------------------------------------------------------------------ #
    # YAML / helpers
    # ------------------------------------------------------------------ #
    def load_config(self):
        path = os.getenv("YML_CONFIG")
        if not path:
            raise RuntimeError("YML_CONFIG not set")
        with open(path, "r", encoding="utf-8") as fh:
            return yaml.safe_load(fh)

    def import_msg_type(self, spec):
        pkg, _, name = spec.partition("/msg/")
        return getattr(importlib.import_module(f"{pkg}.msg"), name)

    def qos_from_dict(self, d):
        rel = {"reliable": QoSReliabilityPolicy.RELIABLE,
               "best_effort": QoSReliabilityPolicy.BEST_EFFORT}[d.get("reliability", "reliable")]
        hist = {"keep_last": QoSHistoryPolicy.KEEP_LAST,
                "keep_all": QoSHistoryPolicy.KEEP_ALL}[d.get("history", "keep_last")]
        dur = {"volatile": QoSDurabilityPolicy.VOLATILE,
               "transient_local": QoSDurabilityPolicy.TRANSIENT_LOCAL}[d.get("durability", "volatile")]
        depth = int(d.get("depth", 1))
        return QoSProfile(reliability=rel, history=hist, durability=dur, depth=depth)

    # ------------------------------------------------------------------ #
    # Topic wiring
    # ------------------------------------------------------------------ #
    def merge_with_defaults(self, entry):
        merged = dict(self.defaults)
        merged.update(entry)
        merged["qos_profile"] = {
            **self.defaults.get("qos_profile", {}),
            **entry.get("qos_profile", {}),
        }
        return merged

    def setup_topic(self, entry):
        cfg = self.merge_with_defaults(entry)
        topic = cfg["topic_name"]
        source = cfg.get("source", "ros")
        msg_cls = self.import_msg_type(cfg["msg_type"])
        qos = self.qos_from_dict(cfg["qos_profile"])
        self.msg_classes[topic] = msg_cls

        if source == "ros":
            # listen, forward out serial
            cb = self.make_ros_in_cb(topic, msg_cls)
            self.create_subscription(msg_cls, topic, cb, qos)
        elif source == "serial":
            # data comes from serial, publish to ROS
            pub = self.create_publisher(msg_cls, topic, qos)
            self.ros_publishers[topic] = pub
        else:
            self.get_logger().warn(f"{topic}: unknown source '{source}'")

    # ------------------------------------------------------------------ #
    # Callbacks
    # ------------------------------------------------------------------ #
    def make_ros_in_cb(self, topic, msg_cls):
        # tiny closure to capture topic name
        def cb(msg):
            # convert ROS msg → dict → JSON
            payload = {
                "topic": topic,
                "data": message_to_ordereddict(msg),
            }
            self.serial_in_pub.publish(String(data=json.dumps(payload)))
        return cb

    def serial_out_cb(self, msg):
        try:
            payload = json.loads(msg.data)
            t = payload["topic"]
            data_dict = payload["data"]
        except (ValueError, KeyError):
            self.get_logger().warn("Bad serial payload")
            return

        pub = self.ros_publishers.get(t)
        if not pub:
            # no publisher set up for this topic
            self.get_logger().warn(f"Unhandled serial topic {t}")
            return

        msg_cls = self.msg_classes[t]
        ros_msg = msg_cls()
        populate_msg_from_dict(ros_msg, data_dict)
        pub.publish(ros_msg)


def main():
    rclpy.init()
    node = SerialBridge()
    try:
        rclpy.spin(node)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
