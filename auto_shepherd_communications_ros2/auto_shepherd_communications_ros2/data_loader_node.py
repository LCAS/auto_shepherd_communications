#!/usr/bin/env python3

import os
import random
import importlib
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy as QoSR, QoSHistoryPolicy as QoSH, QoSDurabilityPolicy as QoSD


class DataLoaderNode(Node):
    def __init__(self):
        super().__init__("data_loader_node")
        cfg = self.load_config()
        self.defaults = cfg.get("defaults", {})
        self.timers = []
        for entry in cfg.get("topics", []):
            self.create_topic(entry)

    def load_config(self):
        # read YAML file set by env var
        path = os.getenv("TOPIC_CONFIG")
        if not path:
            raise RuntimeError("Environment variable YML_CONFIG is not set")
        with open(path, "r", encoding="utf-8") as fh:
            return yaml.safe_load(fh)

    def import_msg_type(self, spec):
        # dynamic import: "std_msgs/msg/String" → class
        pkg, _, name = spec.partition("/msg/")
        module = importlib.import_module(f"{pkg}.msg")
        return getattr(module, name)

    def qos_from_dict(self, d):
        # build QoSProfile from tiny YAML dict
        rel = {"reliable": QoSR.RELIABLE, "best_effort": QoSR.BEST_EFFORT}[d.get("reliability", "reliable")]
        hist = {"keep_last": QoSH.KEEP_LAST, "keep_all": QoSH.KEEP_ALL}[d.get("history", "keep_last")]
        dur = {"volatile": QoSD.VOLATILE, "transient_local": QoSD.TRANSIENT_LOCAL}[d.get("durability", "volatile")]

        depth = int(d.get("depth", 1))
        return QoSProfile(reliability=rel, history=hist, durability=dur, depth=depth)

    def random_value_for_msg(self, cls):
        # craft a random std_msgs value
        msg = cls()
        if not hasattr(msg, "data"):
            return msg
        n = cls.__name__
        if n.endswith("String"):
            msg.data = f"rnd_{random.randint(0, 9999)}"
        elif n.endswith("Bool"):
            msg.data = bool(random.getrandbits(1))
        elif n.endswith(("Int8", "Int16", "Int32", "Int64", "Integer")):
            msg.data = random.randint(-100, 100)
        elif n.endswith(("UInt8", "UInt16", "UInt32", "UInt64")):
            msg.data = random.randint(0, 100)
        elif n.endswith(("Float32", "Float64")):
            msg.data = random.uniform(-100.0, 100.0)
        return msg

    def merge_with_defaults(self, entry):
        # apply global defaults to a single topic stanza
        merged = dict(self.defaults)
        merged.update(entry)
        merged["qos_profile"] = {
            **self.defaults.get("qos_profile", {}),
            **entry.get("qos_profile", {}),
        }
        return merged

    def create_topic(self, entry):
        # create publisher + timer for one topic
        cfg = self.merge_with_defaults(entry)
        topic = cfg["topic_name"]
        msg_cls = self.import_msg_type(cfg["msg_type"])
        qos = self.qos_from_dict(cfg["qos_profile"])
        mode = cfg.get("sample_data", "none")
        rate = float(cfg.get("sample_rate", 1.0)) or 1.0

        pub = self.create_publisher(msg_cls, topic, qos)
        period = 1.0 / rate

        timer = self.create_timer(period, self.make_timer_cb(pub, msg_cls, mode))
        self.timers.append(timer)

    def make_timer_cb(self, pub, msg_cls, mode):
        # one callback generator per topic
        if mode == "random":
            def cb():
                pub.publish(self.random_value_for_msg(msg_cls))
        else:
            def cb():
                pass  # no-op
        return cb


def main():
    rclpy.init()
    node = DataLoaderNode()
    rclpy.spin(node)
    node.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
