
import pyrealsense2 as rs
import numpy as np
import zmq
import cv2
import sys
import time

# --- IMPORTS DE ROS2 ---
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R

class FoundationPoseBridge(Node):
    def __init__(self):
        super().__init__('foundation_pose_bridge')
        
        # 1. Configurar Publicador ROS2
        self.publisher_ = self.create_publisher(PoseStamped, '/foundation_pose/pose', 10)
        self.get_logger().info('🚀 Bridge Iniciado. Publicando en /foundation_pose/pose')

        # 2. Configurar ZMQ (Doble Canal)
        # CAMBIO: Usamos self.zmq_context para no chocar con self.context de ROS
        self.zmq_context = zmq.Context()
        
        # Canal SALIDA (Imágenes hacia Docker) -> Puerto 5555
        self.socket_pub_img = self.zmq_context.socket(zmq.PUB)
        self.socket_pub_img.bind("tcp://*:5555")
        
        # Canal ENTRADA (Poses desde Docker) -> Puerto 5556
        self.socket_sub_pose = self.zmq_context.socket(zmq.SUB)
        self.socket_sub_pose.connect("tcp://localhost:5556")
        self.socket_sub_pose.setsockopt_string(zmq.SUBSCRIBE, "POSE_RESULT")
        
        print("📡 ZMQ: Enviando imágenes en :5555 | Escuchando poses de :5556")

        # 3. Configurar RealSense
        self.W, self.H = 640, 480
        self.FPS = 30
        self.pipe = rs.pipeline()
        self.cfg = rs.config()
        self.cfg.enable_stream(rs.stream.depth, self.W, self.H, rs.format.z16, self.FPS)
        self.cfg.enable_stream(rs.stream.color, self.W, self.H, rs.format.rgb8, self.FPS)
        self.align = rs.align(rs.stream.color)
        
        # Iniciar Cámara
        self.profile = self.pipe.start(self.cfg)

        # --- APLICAR PRESET SHORT RANGE ---
        depth_sensor = self.profile.get_device().first_depth_sensor()
        if depth_sensor.supports(rs.option.visual_preset):
            # Preset 5 suele ser "Short Range" en L515
            depth_sensor.set_option(rs.option.visual_preset, 5) 
            # O intentar manualmente bajar la potencia del láser si es necesario
            # depth_sensor.set_option(rs.option.laser_power, 100) 
            print("Message: L515 configurada en Short Range Preset")

        depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        print(f"📏 ESCALA DETECTADA L515: {self.depth_scale} metros por unidad")
        
        print("📷 RealSense Iniciada.")
        
        # Obtener Intrínsecos (K)
        color_stream = self.profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = color_stream.get_intrinsics()
        self.K_matrix = np.array([
            [intr.fx, 0, intr.ppx], 
            [0, intr.fy, intr.ppy], 
            [0, 0, 1]
        ], dtype=np.float32)

    def run(self):
        try:
            while rclpy.ok():
                # --- A. LEER CÁMARA ---
                frames = self.pipe.wait_for_frames()
                aligned_frames = self.align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()

                if not depth_frame or not color_frame:
                    continue

                # Convertir a numpy
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                # --- B. ENVIAR A DOCKER (ZMQ PUB) ---
                self.socket_pub_img.send_multipart([
                    b"POSE_DATA",
                    np.array(color_image.shape, dtype=np.int32).tobytes(),
                    np.array(depth_image.shape, dtype=np.int32).tobytes(),
                    self.K_matrix.tobytes(),
                    color_image.tobytes(),
                    depth_image.tobytes()
                ])

                # --- C. REVISAR RESPUESTA DE DOCKER (ZMQ SUB - NO BLOQUEANTE) ---
                # Usamos poll con timeout=0 para no frenar la cámara si el docker es lento
                while self.socket_sub_pose.poll(timeout=0):
                    parts = self.socket_sub_pose.recv_multipart()
                    
                    # parts[0] es el topic, parts[1] es la data
                    mat_bytes = parts[1]
                    mat_4x4 = np.frombuffer(mat_bytes, dtype=np.float32).reshape((4, 4))
                    
                    # --- D. PUBLICAR A ROS2 ---
                    self.publish_ros_pose(mat_4x4)

                # Opcional: Visualización Local
                # cv2.imshow('Host View', cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR))
                # if cv2.waitKey(1) & 0xFF == ord('q'):
                #     break

        except KeyboardInterrupt:
            print("\nDeteniendo...")
        finally:
            # Limpieza segura
            if hasattr(self, 'pipe'):
                self.pipe.stop()
            if hasattr(self, 'zmq_context'):
                self.zmq_context.term()
            cv2.destroyAllWindows()

    def publish_ros_pose(self, mat_4x4):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"

        # Posición
        msg.pose.position.x = float(mat_4x4[0, 3])
        msg.pose.position.y = float(mat_4x4[1, 3])
        msg.pose.position.z = float(mat_4x4[2, 3])

        # Rotación
        rotation_matrix = mat_4x4[:3, :3]
        try:
            quat = R.from_matrix(rotation_matrix).as_quat() # x, y, z, w
            msg.pose.orientation.x = float(quat[0])
            msg.pose.orientation.y = float(quat[1])
            msg.pose.orientation.z = float(quat[2])
            msg.pose.orientation.w = float(quat[3])
            
            self.publisher_.publish(msg)
            # print(f"📍 Pose Publicada Z: {msg.pose.position.z:.2f}m") 
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    bridge = FoundationPoseBridge()
    try:
        bridge.run()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()










