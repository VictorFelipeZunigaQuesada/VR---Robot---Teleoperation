#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from scipy.spatial.transform import Rotation as R

class GraspVisualizer(Node):
    def __init__(self):
        super().__init__('grasp_visualizer')

        # Parámetros exactos de tu lógica
        self.base_frame = 'link_base'
        self.object_frame = 'detected_object_frozen'
        self.dims = {'X': 0.16, 'Y': 0.03, 'Z': 0.21}
        self.thickness_range = (0.01, 0.08)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Publicador de los markers que quieres ver
        self.marker_pub = self.create_publisher(MarkerArray, 'grasp_candidates_markers', 10)

        # Timer a 30Hz para refrescar visualización
        self.timer = self.create_timer(0.033, self.loop)

    def loop(self):
        try:
            # Buscamos el objeto
            t = self.tf_buffer.lookup_transform(self.base_frame, self.object_frame, rclpy.time.Time())
            obj_pos = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
            obj_rot = R.from_quat([t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w])
            
            # Generamos los candidatos con TU lógica
            candidates = self.generate_centered_candidates(obj_pos, obj_rot)

            # Publicamos los markers en RViz
            self.publish_markers(candidates)

        except Exception:
            # Si el frame no existe todavía, no hacemos nada
            pass

    def generate_centered_candidates(self, obj_pos, obj_rot):
        candidates = []
        obj_m = obj_rot.as_matrix()
        axis_names = ['X', 'Y', 'Z']

        for i in range(3):
            for sign in [1.0, -1.0]:
                normal_world = obj_m[:, i] * sign
                dist_to_face = self.dims[axis_names[i]] / 2.0
                tcp_pos = obj_pos + (normal_world * dist_to_face)

                z_tcp = -normal_world
                z_tcp /= (np.linalg.norm(z_tcp) + 1e-12)

                other_axes = [idx for idx in range(3) if idx != i]
                for j in other_axes:
                    thickness = self.dims[axis_names[j]]
                    # Filtro de grosor que usas
                    if not (self.thickness_range[0] <= thickness <= self.thickness_range[1]): 
                        continue

                    x_tcp = obj_m[:, j]
                    y_tcp = np.cross(z_tcp, x_tcp)
                    y_tcp /= (np.linalg.norm(y_tcp) + 1e-12)
                    x_tcp = np.cross(y_tcp, z_tcp)

                    grasp_m = np.column_stack((x_tcp, y_tcp, z_tcp))
                    # Rotación de 90 grados en Z que pides
                    grasp_rot = R.from_matrix(grasp_m) * R.from_euler('z', 90, degrees=True) 

                    candidates.append({
                        'pos': tcp_pos,
                        'quat': grasp_rot.as_quat()
                    })
        return candidates

    def publish_markers(self, candidates):
        marker_array = MarkerArray()
        
        for i, c in enumerate(candidates):
            marker = Marker()
            marker.header.frame_id = self.base_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "grasp_visual"
            marker.id = i
            marker.type = Marker.ARROW # Usamos flechas para ver la dirección del gripper
            marker.action = Marker.ADD
            
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = c['pos']
            marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w = c['quat']
            
            # Dimensiones de la flecha
            marker.scale.x = 0.08  # Largo
            marker.scale.y = 0.01  # Ancho
            marker.scale.z = 0.01  # Alto
            
            # Color: Verde semitransparente para no tapar todo
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            
            marker_array.markers.append(marker)
        
        # Limpiar markers antiguos si hubiera menos ahora
        for i in range(len(candidates), 20):
            m = Marker()
            m.header.frame_id = self.base_frame
            m.ns = "grasp_visual"
            m.id = i
            m.action = Marker.DELETE
            marker_array.markers.append(m)

        self.marker_pub.publish(marker_array)

def main():
    rclpy.init()
    node = GraspVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()