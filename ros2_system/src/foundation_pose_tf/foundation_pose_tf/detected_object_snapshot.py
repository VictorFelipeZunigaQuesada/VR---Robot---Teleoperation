#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker


class DetectedObjectSnapshot(Node):
    def __init__(self):
        super().__init__('detected_object_snapshot')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.marker_pub = self.create_publisher(
            Marker,
            '/detected_object_frozen_marker',
            10
        )

        self.snapshot_taken = False
        self.frozen_tf = None
        self.warned = False

        self.timer = self.create_timer(0.1, self.update)

        self.get_logger().info(
            '🧊 DetectedObjectSnapshot activo (esperando ghost_pose)'
        )

    def update(self):
        # =====================================================
        # ESPERAR TRIGGER (ghost_pose)
        # =====================================================
        if not self.snapshot_taken:
            try:
                # ghost_pose solo como trigger
                self.tf_buffer.lookup_transform(
                    'link_base',
                    'ghost_pose',
                    rclpy.time.Time()
                )

                # leer pose ACTUAL del objeto
                t = self.tf_buffer.lookup_transform(
                    'link_base',
                    'detected_object',
                    rclpy.time.Time()
                )

                self.frozen_tf = t
                self.snapshot_taken = True

                self.get_logger().info(
                    '📸 SNAPSHOT tomado: detected_object congelado'
                )

            except Exception:
                if not self.warned:
                    self.get_logger().warn(
                        '⏳ Esperando ghost_pose y detected_object...'
                    )
                    self.warned = True
                return

        # =====================================================
        # PUBLICAR TF CONGELADO (DINÁMICO)
        # =====================================================
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'link_base'
        tf.child_frame_id = 'detected_object_frozen'
        tf.transform = self.frozen_tf.transform

        self.tf_broadcaster.sendTransform(tf)

        # =====================================================
        # PUBLICAR MARKER (CONTINUO)
        # =====================================================
        marker = Marker()
        marker.header.frame_id = 'detected_object_frozen'
        marker.header.stamp = tf.header.stamp
        marker.ns = 'detected_object_frozen'
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD

        marker.mesh_resource = (
            'file:///home/hugo/dev_ws/src/foundation_pose_tf/meshes/model2/model/model2.obj'
        )
        marker.mesh_use_embedded_materials = True

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        marker.color.a = 1.0

        self.marker_pub.publish(marker)


def main():
    rclpy.init()
    node = DetectedObjectSnapshot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
