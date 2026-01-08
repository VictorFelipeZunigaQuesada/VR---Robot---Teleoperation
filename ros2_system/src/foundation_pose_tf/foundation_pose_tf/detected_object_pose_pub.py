#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf2_ros

class DetectedObjectPosePub(Node):
    def __init__(self):
        super().__init__('detected_object_pose_pub')
        self.pub = self.create_publisher(
            PoseStamped,
            '/detected_object_pose',
            10
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.3, self.publish_pose)
        self.get_logger().info('📦 detected_object_pose publisher iniciado')
    
    def publish_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'camera_optical_frame',
                'detected_object',
                rclpy.time.Time()
            )
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'camera_optical_frame'
            pose.pose.position.x = t.transform.translation.x
            pose.pose.position.y = t.transform.translation.y
            pose.pose.position.z = t.transform.translation.z
            pose.pose.orientation = t.transform.rotation
            
            # 🔍 DEBUG: Ver qué se está publicando
            self.get_logger().info(
                f'📍 Publishing | pos: ({pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}, {pose.pose.position.z:.3f}) | '
                f'quat: (x={pose.pose.orientation.x:.3f}, y={pose.pose.orientation.y:.3f}, '
                f'z={pose.pose.orientation.z:.3f}, w={pose.pose.orientation.w:.3f})'
            )
            
            self.pub.publish(pose)
        except Exception as e:
            self.get_logger().warn(f'⚠️ No se pudo obtener transform: {e}')
    
def main():
    rclpy.init()
    node = DetectedObjectPosePub()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
