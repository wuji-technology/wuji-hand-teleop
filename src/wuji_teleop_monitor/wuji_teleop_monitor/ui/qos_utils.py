#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""QoS auto-match utility — shared logic lifted from the recorder ros_node.

All UI subscribers should call match_publisher_qos() instead of hard-coding
BEST_EFFORT, so the subscription QoS adapts to the publisher's policy:
- D405 wrist cameras (RELIABLE) → subscribe RELIABLE for zero dropped frames
- Head camera (RELIABLE) → ditto
- joint_states and similar (BEST_EFFORT) → subscribe BEST_EFFORT for low latency
"""

from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)


def match_publisher_qos(
    node: Node,
    topic: str,
    *,
    depth: int = 1,
    fallback_reliable: bool = False,
) -> QoSProfile:
    """Query publisher QoS for a topic and return a matching subscriber QoS.

    Args:
        node: ROS2 node (used to query publisher info).
        topic: topic name.
        depth: history depth (default 1, suited to real-time display).
        fallback_reliable: when the query fails, fall back to RELIABLE
            (recommended for recording use cases).

    Returns:
        QoSProfile matching the publisher; on query failure, the BEST_EFFORT
        or RELIABLE default.
    """
    try:
        pub_info_list = node.get_publishers_info_by_topic(topic)
        if pub_info_list:
            pub_qos = pub_info_list[0].qos_profile
            matched = QoSProfile(
                reliability=pub_qos.reliability,
                durability=pub_qos.durability,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=depth,
            )
            reliability_str = (
                'RELIABLE'
                if pub_qos.reliability == QoSReliabilityPolicy.RELIABLE
                else 'BEST_EFFORT'
            )
            node.get_logger().info(f'QoS auto-match {topic}: {reliability_str}')
            return matched
    except Exception as exc:
        node.get_logger().debug(f'QoS query failed {topic}: {exc}')

    if fallback_reliable:
        node.get_logger().info(f'QoS fallback {topic}: RELIABLE (publisher not ready)')
        return QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=depth,
        )

    node.get_logger().info(f'QoS fallback {topic}: BEST_EFFORT (publisher not ready)')
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
    )
