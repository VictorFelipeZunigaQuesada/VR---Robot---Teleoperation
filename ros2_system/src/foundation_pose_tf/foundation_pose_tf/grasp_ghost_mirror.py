#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException
from scipy.spatial.transform import Rotation as R


class GraspGhostMirror(Node):

    def __init__(self):
        super().__init__('grasp_ghost_mirror')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(
            MarkerArray,
            'grasp_filters',
            self.callback,
            10
        )

        self.pub = self.create_publisher(
            MarkerArray,
            'ghost_grasp_markers',
            10
        )

        self.base_frame = 'link_base'
        self.object_frame = 'detected_object_frozen'
        self.ghost_frame = 'ghost_pose'

        # -------------------------
        # ESTADO INTERNO (CLAVE)
        # -------------------------
        self.ghost_ready = False
        self.warned_missing_ghost = False

        self.p_ghost = None
        self.r_ghost = None

        self.get_logger().info("🟡 GraspGhostMirror listo (esperando ghost_pose)")


    def callback(self, msg: MarkerArray):

        # -------------------------
        # 1. TF detected_object (OBLIGATORIO)
        # -------------------------
        try:
            t_obj = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.object_frame,
                rclpy.time.Time()
            )
        except TransformException:
            return  # Silencioso, es normal al inicio

        p_obj = np.array([
            t_obj.transform.translation.x,
            t_obj.transform.translation.y,
            t_obj.transform.translation.z
        ])

        r_obj = R.from_quat([
            t_obj.transform.rotation.x,
            t_obj.transform.rotation.y,
            t_obj.transform.rotation.z,
            t_obj.transform.rotation.w
        ])

        # -------------------------
        # 2. SNAPSHOT DEL ghost_pose (UNA SOLA VEZ)
        # -------------------------
        if not self.ghost_ready:
            try:
                t_ghost = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.ghost_frame,
                    rclpy.time.Time()
                )

                self.p_ghost = np.array([
                    t_ghost.transform.translation.x,
                    t_ghost.transform.translation.y,
                    t_ghost.transform.translation.z
                ])

                self.r_ghost = R.from_quat([
                    t_ghost.transform.rotation.x,
                    t_ghost.transform.rotation.y,
                    t_ghost.transform.rotation.z,
                    t_ghost.transform.rotation.w
                ])

                self.ghost_ready = True
                self.get_logger().info("📸 ghost_pose capturado → comenzando proyección de grasps")

            except TransformException:
                if not self.warned_missing_ghost:
                    self.get_logger().warn(
                        "[GraspGhostMirror] ghost_pose aún no existe. Esperando..."
                    )
                    self.warned_missing_ghost = True
                return  # NO procesamos nada aún

        # -------------------------
        # 3. PROYECCIÓN DE GRASPS
        # -------------------------
        out = MarkerArray()
        ghost_id = 0

        for marker in msg.markers:

            if marker.action != Marker.ADD:
                continue

            p_grasp = np.array([
                marker.pose.position.x,
                marker.pose.position.y,
                marker.pose.position.z
            ])

            r_grasp = R.from_quat([
                marker.pose.orientation.x,
                marker.pose.orientation.y,
                marker.pose.orientation.z,
                marker.pose.orientation.w
            ])

            # detected_object → local
            p_local = r_obj.inv().apply(p_grasp - p_obj)
            r_local = r_obj.inv() * r_grasp

            # local → ghost (SNAPSHOT)
            p_g = self.p_ghost + self.r_ghost.apply(p_local)
            r_g = self.r_ghost * r_local

            g = Marker()
            g.header.frame_id = self.base_frame
            g.header.stamp = self.get_clock().now().to_msg()

            g.ns = "ghost_grasps"
            g.id = ghost_id
            g.type = marker.type
            g.action = Marker.ADD

            g.pose.position.x = p_g[0]
            g.pose.position.y = p_g[1]
            g.pose.position.z = p_g[2]

            q = r_g.as_quat()
            g.pose.orientation.x = q[0]
            g.pose.orientation.y = q[1]
            g.pose.orientation.z = q[2]
            g.pose.orientation.w = q[3]

            g.scale = marker.scale

            # Amarillo translúcido
            g.color.r = 1.0
            g.color.g = 1.0
            g.color.b = 0.0
            g.color.a = 0.8

            out.markers.append(g)
            ghost_id += 1

        self._clear_old(out, ghost_id)
        self.pub.publish(out)


    def _clear_old(self, array: MarkerArray, start_id: int):
        for i in range(start_id, start_id + 20):
            m = Marker()
            m.header.frame_id = self.base_frame
            m.ns = "ghost_grasps"
            m.id = i
            m.action = Marker.DELETE
            array.markers.append(m)


def main():
    rclpy.init()
    node = GraspGhostMirror()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
