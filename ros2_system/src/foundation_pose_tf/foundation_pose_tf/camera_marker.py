import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker


class CameraMarker(Node):
    def __init__(self):
        super().__init__('camera_marker')

        self.pub = self.create_publisher(
            Marker,
            '/camera_visualization_marker',
            1
        )

        
        self.timer = self.create_timer(1.0, self.publish_marker)

        self.get_logger().info('📷 Camera marker node started')


    def publish_marker(self):
        marker = Marker()

        marker.header.frame_id = 'camera_link'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'camera'
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD

        marker.mesh_resource = 'package://foundation_pose_tf/meshes/camera/l515.dae'
        
        marker.mesh_use_embedded_materials = True

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = 0.0

        self.pub.publish(marker)



def main():
    rclpy.init()
    node = CameraMarker()
    rclpy.spin(node)
    rclpy.shutdown()
