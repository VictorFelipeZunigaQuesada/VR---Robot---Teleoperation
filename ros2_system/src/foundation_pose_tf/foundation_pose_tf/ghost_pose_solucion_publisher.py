#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations as tft
import numpy as np


class TcpGraspDebugTF(Node):

    def __init__(self):
        super().__init__('ghost_pose_solucion_publisher')

        # Frames existentes
        self.world = 'link_base'
        self.object_frame = 'detected_object_frozen'
        self.tcp_frame = 'tcp_link'
        self.ghost_frame = 'ghost_pose'

        # Frame de salida (SOLO VISUAL)
        self.out_frame = 'tcp_grasp_check'

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.1, self.publish)

        self.get_logger().info(
            '🧠 Publicando tcp_grasp_check (TCP ideal sobre ghost, SOLO TF)'
        )

    def tf_to_mat(self, tf):
        t = tf.transform.translation
        q = tf.transform.rotation
        return tft.concatenate_matrices(
            tft.translation_matrix([t.x, t.y, t.z]),
            tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        )

    def publish(self):
        try:
            T_WO = self.tf_buffer.lookup_transform(
                self.world, self.object_frame, rclpy.time.Time()
            )
            T_WT = self.tf_buffer.lookup_transform(
                self.world, self.tcp_frame, rclpy.time.Time()
            )
            T_WG = self.tf_buffer.lookup_transform(
                self.world, self.ghost_frame, rclpy.time.Time()
            )
        except Exception:
            return

        # Matrices
        M_WO = self.tf_to_mat(T_WO)
        M_WT = self.tf_to_mat(T_WT)
        M_WG = self.tf_to_mat(T_WG)

        # 🔑 GRASP REAL (objeto → TCP)
        M_OT = np.linalg.inv(M_WO) @ M_WT

        # 🔁 Aplicar mismo grasp sobre el ghost
        M_WT_goal = M_WG @ M_OT

        # Extraer resultado
        trans = tft.translation_from_matrix(M_WT_goal)
        quat = tft.quaternion_from_matrix(M_WT_goal)

        # Publicar TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.world
        t.child_frame_id = self.out_frame

        t.transform.translation.x = float(trans[0])
        t.transform.translation.y = float(trans[1])
        t.transform.translation.z = float(trans[2])

        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = TcpGraspDebugTF()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
