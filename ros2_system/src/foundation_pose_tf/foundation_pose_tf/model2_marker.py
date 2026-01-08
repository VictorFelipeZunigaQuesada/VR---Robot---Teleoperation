import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from ament_index_python.packages import get_package_share_directory
import os

class ObjectMarker(Node):
    def __init__(self):
        super().__init__('model_marker')

        self.pub = self.create_publisher(
            Marker,
            '/visualization_marker',
            10
        )

        self.timer = self.create_timer(0.1, self.publish_marker)

        self.get_logger().info('Model marker node started')

    def publish_marker(self):
        marker = Marker()

        # --- HEADER ---
        marker.header.frame_id = 'detected_object'

        # CLAVE: no interpolar en tiempo
        marker.header.stamp.sec = 0
        marker.header.stamp.nanosec = 0

        # --- IDENTIDAD ---
        marker.ns = 'model'
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD

        # --- MESH ---
        marker.mesh_resource = 'package://foundation_pose_tf/meshes/model2/model/model2.obj'
        marker.mesh_use_embedded_materials = True

        # --- POSE (IDENTIDAD, MUY IMPORTANTE) ---
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # --- ESCALA ---
        scale = 1.0
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale

        # --- COLOR ---
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        # --- VIDA INFINITA ---
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        self.pub.publish(marker)


def main():
    rclpy.init()
    node = ObjectMarker()
    rclpy.spin(node)
    rclpy.shutdown()
