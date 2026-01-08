#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from scipy.spatial.transform import Rotation as R


class GhostTableFilter(Node):
    def __init__(self):
        super().__init__('ghost_table_filter')

        self.sub = self.create_subscription(
            MarkerArray, 'ghost_grasp_markers', self.callback, 10
        )
        self.pub = self.create_publisher(
            MarkerArray, 'ghost_grasp_filtered', 10
        )

        # Ajustes
        self.declare_parameter('table_z', 0.0)
        self.declare_parameter('clearance', 0.003)
        self.declare_parameter('axis', 'x')
        self.declare_parameter('use_marker_scale', True)
        self.declare_parameter('fallback_length', 0.06)
        self.declare_parameter('debug', False)

        self.base_frame = 'link_base'

        # -------------------------
        # ESTADO INTERNO (CLAVE)
        # -------------------------
        self.locked = False
        self.cached_markers = None

        self.get_logger().info("🟡 GhostTableFilter listo (esperando primer snapshot)")


    def callback(self, msg: MarkerArray):

        # -------------------------
        # SNAPSHOT ÚNICO
        # -------------------------
        if not self.locked:
            self.cached_markers = msg.markers
            self.locked = True
            self.get_logger().info("📸 Snapshot de ghost_grasp_markers capturado")
        else:
            # Ignorar updates posteriores
            msg = MarkerArray()
            msg.markers = self.cached_markers

        # -------------------------
        # PARÁMETROS
        # -------------------------
        table_z = float(self.get_parameter('table_z').value)
        clearance = float(self.get_parameter('clearance').value)
        axis = str(self.get_parameter('axis').value).lower()
        use_marker_scale = bool(self.get_parameter('use_marker_scale').value)
        fallback_length = float(self.get_parameter('fallback_length').value)
        debug = bool(self.get_parameter('debug').value)

        axis_map = {
            'x': np.array([1.0, 0.0, 0.0]),
            'y': np.array([0.0, 1.0, 0.0]),
            'z': np.array([0.0, 0.0, 1.0]),
        }
        local_axis = axis_map.get(axis, np.array([1.0, 0.0, 0.0]))

        out = MarkerArray()
        valid_id = 0
        filtered_count = 0

        # -------------------------
        # FILTRO GEOMÉTRICO
        # -------------------------
        for marker in msg.markers:
            if marker.action != Marker.ADD:
                continue

            p = np.array([
                marker.pose.position.x,
                marker.pose.position.y,
                marker.pose.position.z
            ])

            r = R.from_quat([
                marker.pose.orientation.x,
                marker.pose.orientation.y,
                marker.pose.orientation.z,
                marker.pose.orientation.w
            ])

            d = r.apply(local_axis)
            d = d / (np.linalg.norm(d) + 1e-12)

            length = fallback_length
            if use_marker_scale and marker.type == Marker.ARROW:
                if marker.scale.x > 1e-6:
                    length = float(marker.scale.x)

            base_pt = p
            tip_pt = p + d * length

            min_z = min(base_pt[2], tip_pt[2])
            hits_table = min_z < (table_z + clearance)

            if hits_table:
                filtered_count += 1
                continue

            m = Marker()
            m.header.frame_id = self.base_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "ghost_grasp_filtered"
            m.id = valid_id
            m.type = marker.type
            m.action = Marker.ADD
            m.pose = marker.pose
            m.scale = marker.scale
            m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 1.0

            out.markers.append(m)
            valid_id += 1

        self._clear_old(out, valid_id)
        self.pub.publish(out)

        if debug:
            self.get_logger().info(
                f"GhostTableFilter: in={len(msg.markers)} out={valid_id} filtered={filtered_count}"
            )


    def _clear_old(self, array: MarkerArray, start_id: int):
        for i in range(start_id, start_id + 40):
            m = Marker()
            m.header.frame_id = self.base_frame
            m.ns = "ghost_grasp_filtered"
            m.id = i
            m.action = Marker.DELETE
            array.markers.append(m)


def main():
    rclpy.init()
    node = GhostTableFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
