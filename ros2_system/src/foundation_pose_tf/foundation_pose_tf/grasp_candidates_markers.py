#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R

class GraspGeometricFilter(Node):

    def __init__(self):
        super().__init__('grasp_candidates_markers')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(
            MarkerArray,
            'grasp_candidates_markers',
            self.callback,
            10
        )

        self.pub = self.create_publisher(
            MarkerArray,
            'grasp_filters',
            10
        )

        self.base_frame = 'link_base'
        self.object_frame = 'detected_object_frozen'

        self.get_logger().info(
            '🟢 GraspGeometricFilter listo (usando detected_object_frozen)'
        )

    def callback(self, msg: MarkerArray):

        # -----------------------------------
        # TF del objeto CONGELADO
        # -----------------------------------
        if not self.tf_buffer.can_transform(
            self.base_frame,
            self.object_frame,
            rclpy.time.Time()
        ):
            return  # silencio total

        t = self.tf_buffer.lookup_transform(
            self.base_frame,
            self.object_frame,
            rclpy.time.Time()
        )

        obj_pos = np.array([
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z
        ])

        obj_rot = R.from_quat([
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w
        ])

        # Vector "abajo" del mundo en frame del objeto
        world_down = np.array([0.0, 0.0, -1.0])
        down_obj = obj_rot.inv().apply(world_down)

        filtered = MarkerArray()
        valid_id = 0

        for marker in msg.markers:
            if marker.action != Marker.ADD:
                continue

            m_pos = np.array([
                marker.pose.position.x,
                marker.pose.position.y,
                marker.pose.position.z
            ])

            v = m_pos - obj_pos
            norm = np.linalg.norm(v)
            if norm < 1e-6:
                continue

            v /= norm
            v_obj = obj_rot.inv().apply(v)

            # ❌ grasps hacia la cara apoyada
            if np.dot(v_obj, down_obj) > 0.7:
                continue

            f = Marker()
            f.header = marker.header
            f.pose = marker.pose
            f.scale = marker.scale
            f.type = marker.type
            f.action = Marker.ADD

            f.ns = "grasp_filters"
            f.id = valid_id
            f.color.r = 0.0
            f.color.g = 1.0
            f.color.b = 1.0
            f.color.a = 1.0

            filtered.markers.append(f)
            valid_id += 1

        self._clear_old_markers(filtered, valid_id)
        self.pub.publish(filtered)
    
    def _clear_old_markers(self, array: MarkerArray, start_id: int):
        for i in range(start_id, start_id + 20):
            m = Marker()
            m.header.frame_id = self.base_frame
            m.ns = "grasp_filters"
            m.id = i
            m.action = Marker.DELETE
            array.markers.append(m)



def main():
    rclpy.init()
    node = GraspGeometricFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
