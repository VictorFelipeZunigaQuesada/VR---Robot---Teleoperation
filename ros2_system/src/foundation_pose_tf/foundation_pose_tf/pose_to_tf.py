#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

class PoseToTF(Node):
    def __init__(self):
        super().__init__('pose_to_tf')

        self.last_pose = None

        self.create_subscription(
            PoseStamped,
            '/foundation_pose/pose',
            self.pose_callback,
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.publish_tf)

        self.get_logger().info(
            'pose_to_tf started (FoundationPose → TF using camera_optical_frame)'
        )

    def pose_callback(self, msg: PoseStamped):
        self.last_pose = msg

    def publish_tf(self):
        if self.last_pose is None:
            return

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()

        # ✅ Frame ÓPTICO correcto (el que tú definiste)
        t.header.frame_id = 'camera_optical_frame'
        t.child_frame_id = 'detected_object'

        # 🔹 Traslación TAL CUAL viene de FoundationPose
        p = self.last_pose.pose.position
        t.transform.translation.x = p.x
        t.transform.translation.y = p.y
        t.transform.translation.z = p.z

        # 🔹 Rotación TAL CUAL viene de FoundationPose
        q = self.last_pose.pose.orientation
        t.transform.rotation.x = q.x
        t.transform.rotation.y = q.y
        t.transform.rotation.z = q.z
        t.transform.rotation.w = q.w

        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = PoseToTF()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
