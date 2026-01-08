#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException
from scipy.spatial.transform import Rotation as R


class GhostToDetectedMirror(Node):

    def __init__(self):
        super().__init__('ghost_to_detected_mirror')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(
            MarkerArray,
            'ghost_grasp_filtered',
            self.callback,
            10
        )

        self.pub = self.create_publisher(
            MarkerArray,
            'detected_grasp_valid',
            10
        )

        self.base_frame = 'link_base'
        self.ghost_frame = 'ghost_pose'
        self.object_frame = 'detected_object_frozen'

        # -------------------------
        # ESTADO INTERNO
        # -------------------------
        self.ready = False
        self.warned = False

        self.p_ghost = None
        self.r_ghost = None

        self.p_obj = None
        self.r_obj = None

        self.get_logger().info(
            "🟡 GhostToDetectedMirror listo (esperando ghost_pose + detected_object_frozen)"
        )


    def callback(self, msg: MarkerArray):

        # -------------------------
        # 1. SNAPSHOT (UNA SOLA VEZ)
        # -------------------------
        if not self.ready:
            try:
                # ghost_pose
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

                # detected_object
                t_obj = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.object_frame,
                    rclpy.time.Time()
                )

                self.p_obj = np.array([
                    t_obj.transform.translation.x,
                    t_obj.transform.translation.y,
                    t_obj.transform.translation.z
                ])

                self.r_obj = R.from_quat([
                    t_obj.transform.rotation.x,
                    t_obj.transform.rotation.y,
                    t_obj.transform.rotation.z,
                    t_obj.transform.rotation.w
                ])

                self.ready = True
                self.get_logger().info(
                    "📸 Snapshot capturado (ghost_pose + detected_object_frozen)"
                )

            except TransformException:
                if not self.warned:
                    self.get_logger().warn(
                        "[GhostToDetectedMirror] Esperando ghost_pose / detected_object_frozen..."
                    )
                    self.warned = True
                return

        # -------------------------
        # 2. PROYECCIÓN (ESTABLE)
        # -------------------------
        out = MarkerArray()
        valid_id = 0

        for marker in msg.markers:
            if marker.action != Marker.ADD:
                continue

            p_g = np.array([
                marker.pose.position.x,
                marker.pose.position.y,
                marker.pose.position.z
            ])

            r_g = R.from_quat([
                marker.pose.orientation.x,
                marker.pose.orientation.y,
                marker.pose.orientation.z,
                marker.pose.orientation.w
            ])

            # ghost → local
            p_local = self.r_ghost.inv().apply(p_g - self.p_ghost)
            r_local = self.r_ghost.inv() * r_g

            # local → detected (SNAPSHOT)
            p_d = self.p_obj + self.r_obj.apply(p_local)
            r_d = self.r_obj * r_local

            m = Marker()
            m.header.frame_id = self.base_frame
            m.header.stamp = self.get_clock().now().to_msg()

            m.ns = "detected_grasp_valid"
            m.id = valid_id
            m.type = marker.type
            m.action = Marker.ADD

            m.pose.position.x = p_d[0]
            m.pose.position.y = p_d[1]
            m.pose.position.z = p_d[2]

            q = r_d.as_quat()
            m.pose.orientation.x = q[0]
            m.pose.orientation.y = q[1]
            m.pose.orientation.z = q[2]
            m.pose.orientation.w = q[3]

            m.scale = marker.scale

            # Azul = grasps finales
            m.color.r = 0.0
            m.color.g = 0.4
            m.color.b = 1.0
            m.color.a = 1.0

            out.markers.append(m)
            valid_id += 1

        self._clear_old(out, valid_id)
        self.pub.publish(out)


    def _clear_old(self, array: MarkerArray, start_id: int):
        for i in range(start_id, start_id + 30):
            m = Marker()
            m.header.frame_id = self.base_frame
            m.ns = "detected_grasp_valid"
            m.id = i
            m.action = Marker.DELETE
            array.markers.append(m)


def main():
    rclpy.init()
    node = GhostToDetectedMirror()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
