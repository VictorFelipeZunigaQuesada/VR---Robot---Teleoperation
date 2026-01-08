#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TransformStamped
from visualization_msgs.msg import Marker
import tf2_ros
from tf_transformations import quaternion_from_euler

class UnityPosePublisher(Node):
    def __init__(self):
        super().__init__('unity_pose_publisher')
        
        # Subscriber para la pose de Unity
        self.subscription = self.create_subscription(
            Pose,
            '/target_pose',
            self.pose_callback,
            10
        )
        
        # Broadcaster para TF dinámicos
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Broadcaster para TF estáticos
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        
        # Publisher para el marker
        self.marker_pub = self.create_publisher(Marker, '/unity_object_marker', 10)
        
        # Publicar el frame offset estático una vez
        self.publish_static_offset()
        
        self.get_logger().info(
            '🟢 Publicando: unity_pose → offset (90° Z + 90° X) → ghost_pose → Marker'
        )

    def publish_static_offset(self):
        """Publica el frame offset estático con rotación 90° en Z y 90° en X"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'unity_pose'
        t.child_frame_id = 'offset'
        
        # Sin traslación
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        # Rotación: roll=90°(X), pitch=0°(Y), yaw=90°(Z)
        # Orden: yaw (Z), pitch (Y), roll (X) en radianes
        roll = 1.5708   # 90° en X
        pitch = 0.0     # 0° en Y
        yaw = 1.5708    # 90° en Z
        
        quat = quaternion_from_euler(roll, pitch, yaw)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        
        self.static_tf_broadcaster.sendTransform(t)

    def pose_callback(self, msg: Pose):
        now = self.get_clock().now().to_msg()
        
        # =====================================================
        # 1️⃣ FRAME: unity_pose (SOLO TRASLACIÓN)
        # Padre: link_base
        # =====================================================
        t1 = TransformStamped()
        t1.header.stamp = now
        t1.header.frame_id = 'link_base'
        t1.child_frame_id = 'unity_pose'
        
        # TRASLACIÓN: Unity → ROS
        t1.transform.translation.x =  msg.position.z   # Z_unity → X_ros
        t1.transform.translation.y = -msg.position.x   # -X_unity → Y_ros
        t1.transform.translation.z =  msg.position.y   # Y_unity → Z_ros
        
        # Sin rotación
        t1.transform.rotation.w = 1.0
        t1.transform.rotation.x = 0.0
        t1.transform.rotation.y = 0.0
        t1.transform.rotation.z = 0.0
        
        self.tf_broadcaster.sendTransform(t1)
        
        # =====================================================
        # 2️⃣ FRAME: ghost_pose (ROTACIONES DE UNITY DIRECTAS)
        # Padre: offset
        # =====================================================
        t3 = TransformStamped()
        t3.header.stamp = now
        t3.header.frame_id = 'offset'
        t3.child_frame_id = 'ghost_pose'
        
        # Sin traslación
        t3.transform.translation.x = 0.0
        t3.transform.translation.y = 0.0
        t3.transform.translation.z = 0.0
        
        # Rotaciones de Unity SIN CONVERSIÓN (directas)
        t3.transform.rotation.x = -msg.orientation.x
        t3.transform.rotation.y = msg.orientation.y
        t3.transform.rotation.z = msg.orientation.z
        t3.transform.rotation.w = -msg.orientation.w
        
        self.tf_broadcaster.sendTransform(t3)
        
        # =====================================================
        # 3️⃣ PUBLICAR MARKER en ghost_pose
        # =====================================================
        self.publish_ghost_marker(now)

    def publish_ghost_marker(self, stamp):
        """Publica un marker 3D en el frame ghost_pose"""
        marker = Marker()
        marker.header.frame_id = "ghost_pose"
        marker.header.stamp = stamp
        marker.ns = "ghost_object"
        marker.id = 99
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        
        # Pose del marker (identidad)
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        
        # Ruta al mesh
        marker.mesh_resource = 'file:///home/hugo/dev_ws/src/foundation_pose_tf/meshes/model2/model/model2.obj'
        
        # Escala
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        
        # Color (semi-transparente amarillo)
        marker.color.a = 0.7
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        
        # Usar materiales embebidos del mesh
        marker.mesh_use_embedded_materials = True
        
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = UnityPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()