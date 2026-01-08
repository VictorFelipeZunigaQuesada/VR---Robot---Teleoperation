import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker


class ObjectMarker(Node):
    def __init__(self):
        super().__init__('object_marker')

        self.pub = self.create_publisher(
            Marker,
            '/visualization_marker',
            10
        )

        # 10 Hz está perfecto
        self.timer = self.create_timer(0.1, self.publish_marker)

        self.get_logger().info('🧈 Object marker node started')


    def publish_marker(self):
        marker = Marker()

        # --- HEADER ---
        marker.header.frame_id = 'detected_object'
        marker.header.stamp = self.get_clock().now().to_msg()

        # --- IDENTIDAD ---
        marker.ns = 'butter'
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD

        # --- MESH (RUTA ABSOLUTA CORRECTA) ---
        marker.mesh_resource = (
            'file:///home/hugo/dev_ws/src/foundation_pose_tf/meshes/'
            'Butter-20251226T194747Z-3-001/Butter/google_16k/'
            'textured_simple.obj'
        )
        marker.mesh_use_embedded_materials = True

        # --- ESCALA (por ahora 1.0, luego la ajustamos) ---
        scale = 0.01
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale

        # --- COLOR (OBLIGATORIO AUNQUE HAYA TEXTURA) ---
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        self.pub.publish(marker)


def main():
    rclpy.init()
    node = ObjectMarker()
    rclpy.spin(node)
    rclpy.shutdown()
