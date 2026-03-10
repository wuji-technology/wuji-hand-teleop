#!/usr/bin/env python3
"""
MANUS raw_nodes to MediaPipe Hand Landmarks converter with Open3D visualization.

MANUS 25 nodes -> MediaPipe 21 landmarks

Key findings:
- Node 1 (Index MCP) is at origin (0,0,0) - used as WRIST
- Other MCP nodes have fixed positions relative to the palm
- Need to flip X axis to correct mirror issue
- Skip nodes 9, 14, 19 (DIP in MANUS) to map 5 nodes to 4 MediaPipe landmarks

MANUS structure (by node_id):
    Index:  1(MCP=origin) -> 2(PIP) -> 3(IP) -> 4(DIP) -> 5(TIP)
    Middle: 6(MCP) -> 7(PIP) -> 8(IP) -> 9(DIP) -> 10(TIP)
    Ring:   11(MCP) -> 12(PIP) -> 13(IP) -> 14(DIP) -> 15(TIP)
    Pinky:  16(MCP) -> 17(PIP) -> 18(IP) -> 19(DIP) -> 20(TIP)
    Thumb:  21(MCP) -> 22(PIP) -> 23(DIP) -> 24(TIP)
    Invalid: 25

MediaPipe mapping:
    WRIST: node 1
    Index:  nodes 2,3,4,5 -> MCP,PIP,DIP,TIP
    Middle: nodes 6,7,8,10 -> MCP,PIP,DIP,TIP (skip 9)
    Ring:   nodes 11,12,13,15 -> MCP,PIP,DIP,TIP (skip 14)
    Pinky:  nodes 16,17,18,20 -> MCP,PIP,DIP,TIP (skip 19)
    Thumb:  nodes 21,22,23,24 -> CMC,MCP,IP,TIP
"""

import sys
import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from manus_ros2_msgs.msg import ManusGlove

# MediaPipe landmark indices
WRIST = 0
THUMB_CMC = 1
THUMB_MCP = 2
THUMB_IP = 3
THUMB_TIP = 4
INDEX_MCP = 5
INDEX_PIP = 6
INDEX_DIP = 7
INDEX_TIP = 8
MIDDLE_MCP = 9
MIDDLE_PIP = 10
MIDDLE_DIP = 11
MIDDLE_TIP = 12
RING_MCP = 13
RING_PIP = 14
RING_DIP = 15
RING_TIP = 16
PINKY_MCP = 17
PINKY_PIP = 18
PINKY_DIP = 19
PINKY_TIP = 20

# Direct mapping from MANUS node_id to MediaPipe landmark index
# Node 1 (Index MCP) is at origin (0,0,0) - it's the WRIST/root
MANUS_TO_MEDIAPIPE = {
    # Node 1 is the root/WRIST (at origin 0,0,0)
    1: WRIST,

    # Index: nodes 2-5 -> MediaPipe INDEX (node 1 used as WRIST)
    # All 4 remaining Index nodes map to 4 MediaPipe Index landmarks
    2: INDEX_MCP,
    #3: INDEX_PIP,
    4: INDEX_DIP,
    5: INDEX_TIP,

    # Middle: skip DIP node 9 (MANUS has 5 nodes, MediaPipe needs 4)
    6: MIDDLE_MCP,  # Hidden
    #7: MIDDLE_PIP,
    8: MIDDLE_DIP,
    9: MIDDLE_DIP,
    10: MIDDLE_TIP,

    # Ring: skip DIP node 14
    11: RING_MCP,  # Hidden
    #12: RING_PIP,
    13: RING_DIP,
    14: RING_DIP,
    15: RING_TIP,

    # Pinky: skip DIP node 19
    # 16: PINKY_MCP,  # Hidden
    17: PINKY_PIP,
    18: PINKY_DIP,
    # 19: skip (Pinky DIP in MANUS)
    20: PINKY_TIP,

    # Thumb: MANUS has 4 nodes (21-24), MediaPipe needs 4 (CMC,MCP,IP,TIP)
    # Perfect 1:1 mapping
    21: THUMB_CMC,  # Hidden - 离掌根最近，不动
    22: THUMB_MCP,
    23: THUMB_IP,
    24: THUMB_TIP,

    # Node 25 (Invalid) - skip
}

# MediaPipe hand connections
MEDIAPIPE_CONNECTIONS = [
    (WRIST, THUMB_CMC), (THUMB_CMC, THUMB_MCP), (THUMB_MCP, THUMB_IP), (THUMB_IP, THUMB_TIP),
    (WRIST, INDEX_MCP), (INDEX_MCP, INDEX_PIP), (INDEX_PIP, INDEX_DIP), (INDEX_DIP, INDEX_TIP),
    (WRIST, MIDDLE_MCP), (MIDDLE_MCP, MIDDLE_PIP), (MIDDLE_PIP, MIDDLE_DIP), (MIDDLE_DIP, MIDDLE_TIP),
    (WRIST, RING_MCP), (RING_MCP, RING_PIP), (RING_PIP, RING_DIP), (RING_DIP, RING_TIP),
    (WRIST, PINKY_MCP), (PINKY_MCP, PINKY_PIP), (PINKY_PIP, PINKY_DIP), (PINKY_DIP, PINKY_TIP),
]

LANDMARK_NAMES = [
    "WRIST", "THUMB_CMC", "THUMB_MCP", "THUMB_IP", "THUMB_TIP",
    "INDEX_MCP", "INDEX_PIP", "INDEX_DIP", "INDEX_TIP",
    "MIDDLE_MCP", "MIDDLE_PIP", "MIDDLE_DIP", "MIDDLE_TIP",
    "RING_MCP", "RING_PIP", "RING_DIP", "RING_TIP",
    "PINKY_MCP", "PINKY_PIP", "PINKY_DIP", "PINKY_TIP"
]

FINGER_COLORS = {
    'wrist': [0.5, 0.5, 0.5],
    'thumb': [1.0, 0.0, 0.0],
    'index': [1.0, 0.5, 0.0],
    'middle': [1.0, 1.0, 0.0],
    'ring': [0.0, 1.0, 0.0],
    'pinky': [0.0, 0.5, 1.0],
}


class MediaPipeHandViz:
    """Open3D visualization for MediaPipe hand landmarks."""

    def __init__(self, side="Unknown"):
        self.viz = o3d.visualization.Visualizer()
        self.viz.create_window(window_name=f"MediaPipe Hand ({side})", width=800, height=600)

        self.frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
        self.viz.add_geometry(self.frame)

        self.landmark_meshes = []
        for i in range(21):
            sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.006)
            sphere.compute_vertex_normals()
            sphere.paint_uniform_color(self._get_landmark_color(i))
            self.landmark_meshes.append(sphere)
            self.viz.add_geometry(sphere)

        self.line_set = o3d.geometry.LineSet()
        self.viz.add_geometry(self.line_set)
        self.viz.get_view_control().set_zoom(0.5)

    def _get_landmark_color(self, idx):
        if idx == 0:
            return FINGER_COLORS['wrist']
        elif 1 <= idx <= 4:
            return FINGER_COLORS['thumb']
        elif 5 <= idx <= 8:
            return FINGER_COLORS['index']
        elif 9 <= idx <= 12:
            return FINGER_COLORS['middle']
        elif 13 <= idx <= 16:
            return FINGER_COLORS['ring']
        else:
            return FINGER_COLORS['pinky']

    def update(self, landmarks: np.ndarray):
        for i, sphere in enumerate(self.landmark_meshes):
            pos = landmarks[i]
            sphere.translate(-np.asarray(sphere.get_center()), relative=True)
            sphere.translate(pos, relative=False)
            self.viz.update_geometry(sphere)

        line_points = []
        line_indices = []
        line_colors = []

        for start_idx, end_idx in MEDIAPIPE_CONNECTIONS:
            start_pos = landmarks[start_idx]
            end_pos = landmarks[end_idx]
            idx = len(line_points)
            line_points.append(start_pos)
            line_points.append(end_pos)
            line_indices.append([idx, idx + 1])
            line_colors.append(self._get_landmark_color(end_idx))

        if line_points:
            self.line_set.points = o3d.utility.Vector3dVector(line_points)
            self.line_set.lines = o3d.utility.Vector2iVector(line_indices)
            self.line_set.colors = o3d.utility.Vector3dVector(line_colors)
            self.viz.update_geometry(self.line_set)

    def poll(self):
        self.viz.poll_events()
        self.viz.update_renderer()


class ManusToMediaPipeConverter(Node):
    """Converts MANUS raw_nodes to MediaPipe hand landmarks format."""

    def __init__(self, glove_index=0, flip_x=True):
        super().__init__("manus_to_mediapipe")

        self.glove_index = glove_index
        self.flip_x = flip_x  # Flip X axis to correct mirror
        self.viz = None
        self.latest_landmarks = None
        self.first_message = True

        topic_name = f"/manus_glove_{glove_index}"
        self.subscription = self.create_subscription(
            ManusGlove, topic_name, self.glove_callback, 10
        )
        self.timer = self.create_timer(0.02, self.timer_callback)

        self.get_logger().info(f"Subscribed to {topic_name}")
        self.get_logger().info(f"Flip X axis: {self.flip_x}")

    def print_mapping_info(self, msg: ManusGlove):
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"MANUS -> MediaPipe Mapping (Side: {msg.side})")
        self.get_logger().info(f"{'='*60}")

        for node in msg.raw_nodes:
            nid = node.node_id
            pos = node.pose.position
            if nid in MANUS_TO_MEDIAPIPE:
                mp_idx = MANUS_TO_MEDIAPIPE[nid]
                mp_name = LANDMARK_NAMES[mp_idx]
                self.get_logger().info(
                    f"  MANUS {nid:2d} -> MP {mp_idx:2d} ({mp_name:12s}) "
                    f"pos=({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})"
                )
            else:
                self.get_logger().info(
                    f"  MANUS {nid:2d} -> SKIP "
                    f"pos=({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})"
                )
        self.get_logger().info(f"{'='*60}\n")

    def convert_to_mediapipe(self, msg: ManusGlove) -> np.ndarray:
        landmarks = np.zeros((21, 3), dtype=np.float32)

        for node in msg.raw_nodes:
            nid = node.node_id

            # Get position and apply flip if needed
            x = node.pose.position.x
            y = node.pose.position.y
            z = node.pose.position.z

            # Flip X axis to correct mirror issue
            if self.flip_x:
                x = -x

            pos = np.array([x, y, z])

            if nid in MANUS_TO_MEDIAPIPE:
                mp_idx = MANUS_TO_MEDIAPIPE[nid]
                landmarks[mp_idx] = pos

        return landmarks

    def glove_callback(self, msg: ManusGlove):
        if self.first_message:
            self.print_mapping_info(msg)
            self.viz = MediaPipeHandViz(side=msg.side)
            self.first_message = False

        self.latest_landmarks = self.convert_to_mediapipe(msg)

    def timer_callback(self):
        if self.viz is not None and self.latest_landmarks is not None:
            self.viz.update(self.latest_landmarks)
            self.viz.poll()


def main():
    rclpy.init(args=sys.argv)

    glove_index = 0
    flip_x = True  # Default: flip X to correct mirror

    if len(sys.argv) > 1:
        glove_index = int(sys.argv[1])
    if len(sys.argv) > 2:
        flip_x = sys.argv[2].lower() != 'noflip'

    print(f"\n{'='*60}")
    print("MANUS to MediaPipe Converter")
    print(f"{'='*60}")
    print(f"Glove index: {glove_index}")
    print(f"Flip X axis: {flip_x}")
    print(f"{'='*60}")
    print("\nKey changes:")
    print("  - Node 25 (Invalid) -> WRIST")
    print("  - Flip X axis to correct mirror")
    print(f"\nUsage: python3 {sys.argv[0]} [glove_index] [noflip]")
    print(f"{'='*60}\n")

    converter = ManusToMediaPipeConverter(glove_index, flip_x)

    try:
        rclpy.spin(converter)
    except KeyboardInterrupt:
        pass
    finally:
        converter.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
