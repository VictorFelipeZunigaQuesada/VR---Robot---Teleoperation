#!/usr/bin/env python3
import os
import yaml

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject

from tf2_ros import Buffer, TransformListener, TransformException
from ament_index_python.packages import get_package_share_directory
from tf_transformations import quaternion_from_euler


class SceneManager(Node):
    """
    SceneManager FINAL (PASIVO)

    Responsabilidades:
    Mesa (ONE SHOT)
    Cámara (dinámica)
    """

    def __init__(self):
        super().__init__('scene_manager')

        # ---------------- Frames / IDs ----------------
        self.world_frame = 'link_base'
        self.table_id = 'unity_table'
        self.camera_id = 'physical_camera'

        # ---------------- Publisher ----------------
        self.collision_pub = self.create_publisher(
            CollisionObject,
            '/collision_object',
            10
        )

        # ---------------- TF ----------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------------- Config ----------------
        self.load_config()

        # ---------------- Timers ----------------
        # Mesa: ONE SHOT
        self.table_timer = self.create_timer(1.0, self.publish_table_once)
        # Cámara: dinámica
        self.camera_timer = self.create_timer(0.5, self.publish_camera)

        self.get_logger().info('🟢 SceneManager PASIVO listo (mesa + cámara)')

    # ==================================================
    # ---------------- CONFIG ---------------------------
    # ==================================================
    def load_config(self):
        try:
            pkg = get_package_share_directory('foundation_pose_tf')
            cfg_path = os.path.join(pkg, 'config', 'scene_objects.yaml')
            with open(cfg_path, 'r') as f:
                self.config = yaml.safe_load(f) or {}
            self.get_logger().info(f'Config cargada: {cfg_path}')
        except Exception as e:
            self.get_logger().error(f' Error cargando config: {e}')
            self.config = {}

    # ==================================================
    # ---------------- HELPERS --------------------------
    # ==================================================
    def publish_add(self, obj_id: str, primitive: SolidPrimitive, pose: Pose):
        co = CollisionObject()
        co.header.frame_id = self.world_frame
        co.id = obj_id
        co.operation = CollisionObject.ADD
        co.primitives = [primitive]
        co.primitive_poses = [pose]
        self.collision_pub.publish(co)

    def lookup_pose(self, target_frame: str) -> Pose:
        tf = self.tf_buffer.lookup_transform(
            self.world_frame,
            target_frame,
            rclpy.time.Time()
        )
        pose = Pose()
        pose.position.x = tf.transform.translation.x
        pose.position.y = tf.transform.translation.y
        pose.position.z = tf.transform.translation.z
        pose.orientation = tf.transform.rotation
        return pose

    # ==================================================
    # ---------------- MESA -----------------------------
    # ==================================================
    def publish_table_once(self):
        self.table_timer.cancel()

        table_cfg = self.config.get('table', {})
        dims = table_cfg.get('dimensions', [1.0, 1.0, 0.5])
        pos = table_cfg.get('pose', {}).get('position', [0.0, 0.0, -0.25])
        ori = table_cfg.get('pose', {}).get('orientation', [0.0, 0.0, 0.0])

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [float(d) for d in dims]

        pose = Pose()
        pose.position.x = float(pos[0])
        pose.position.y = float(pos[1])
        pose.position.z = float(pos[2])

        q = quaternion_from_euler(
            float(ori[0]),
            float(ori[1]),
            float(ori[2])
        )
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        self.publish_add(self.table_id, box, pose)
        self.get_logger().info('🟧 Mesa publicada')

    # ==================================================
    # ---------------- CÁMARA ---------------------------
    # ==================================================
    def publish_camera(self):
        try:
            pose = self.lookup_pose('camera_link')
            cam_cfg = self.config.get('camera', {}).get('dimensions', {})
            height = float(cam_cfg.get('height', 0.03))
            radius = float(cam_cfg.get('radius', 0.03))

            cyl = SolidPrimitive()
            cyl.type = SolidPrimitive.CYLINDER
            cyl.dimensions = [height, radius]

            self.publish_add(self.camera_id, cyl, pose)
        except TransformException:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = SceneManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
