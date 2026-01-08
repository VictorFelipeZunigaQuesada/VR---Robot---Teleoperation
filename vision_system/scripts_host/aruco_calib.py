import pyrealsense2 as rs
import numpy as np
import cv2
import cv2.aruco as aruco
from scipy.spatial.transform import Rotation as R

# ==========================================
# 📏 TUS MEDICIONES MANUALES (Lab Setup)
# ==========================================
# Distancia Base -> ArUco Ref (en metros)
# Configuración: Base del robot en (0,0,0). ArUco pegado a 20cm en X negativo.
MANUAL_OFFSET_POS = [0.0, -0.445, 0.0] 

# Rotación relativa: Asumimos que ambos están planos sobre la misma mesa
MANUAL_OFFSET_EULER = [0, 0, 0] # Roll, Pitch, Yaw

# Configuración ArUco
MARKER_SIZE = 0.1 # 10cm (Asegúrate que este sea el tamaño real impreso)
ID_REF = 0         # ID del marcador de referencia

# ==========================================
# ⚙️ INICIALIZACIÓN REALSENSE
# ==========================================
pipe = rs.pipeline()
cfg = rs.config()
# Usamos el stream de color estándar
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

print("📡 Iniciando L515...")
profile = pipe.start(cfg)

# OBTENER INTRÍNSECOS REALES (Directo del sensor)
stream_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
intrinsics = stream_profile.get_intrinsics()

# Construir Matriz K
cam_K = np.array([
    [intrinsics.fx, 0, intrinsics.ppx],
    [0, intrinsics.fy, intrinsics.ppy],
    [0, 0, 1]
], dtype=np.float32)

# Obtener coeficientes de distorsión
dist_coeffs = np.array(intrinsics.coeffs)

print(f"🔍 Intrínsecos cargados: fx={intrinsics.fx:.1f}, fy={intrinsics.fy:.1f}")

# ==========================================
# 🧮 FUNCIONES MATEMÁTICAS
# ==========================================
def get_matrix_from_pose(pos, euler):
    """Convierte posición + euler a Matriz 4x4"""
    mat = np.eye(4)
    mat[:3, 3] = pos
    r = R.from_euler('xyz', euler, degrees=True)
    mat[:3, :3] = r.as_matrix()
    return mat

def inverse_transform(T):
    """Invierte una matriz de transformación homogénea"""
    R_mat = T[:3, :3]
    t_vec = T[:3, 3]
    T_inv = np.eye(4)
    T_inv[:3, :3] = R_mat.T
    T_inv[:3, 3] = -R_mat.T @ t_vec
    return T_inv

# Matriz Base -> Ref (Fija por tu medición manual)
T_base_ref = get_matrix_from_pose(MANUAL_OFFSET_POS, MANUAL_OFFSET_EULER)

# Configuración Detector ArUco
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
aruco_params = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, aruco_params)

# Puntos 3D del marcador (Esquinas)
marker_points = np.array([
    [-MARKER_SIZE/2, MARKER_SIZE/2, 0],
    [MARKER_SIZE/2, MARKER_SIZE/2, 0],
    [MARKER_SIZE/2, -MARKER_SIZE/2, 0],
    [-MARKER_SIZE/2, -MARKER_SIZE/2, 0]
], dtype=np.float32)

print("\n📷 Apunta la L515 al marcador de referencia...")
print("   (Presiona 'q' para salir)")

try:
    while True:
        # 1. Obtener frames de RealSense
        frames = pipe.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame: continue

        # Convertir a array numpy
        frame = np.asanyarray(color_frame.get_data())

        # 2. Detectar
        corners, ids, _ = detector.detectMarkers(frame)
        
        if ids is not None and ID_REF in ids:
            idx = np.where(ids == ID_REF)[0][0]
            
            # 3. Pose del Marcador respecto a la Cámara (solvePnP)
            success, rvec, tvec = cv2.solvePnP(marker_points, corners[idx], cam_K, dist_coeffs)
            
            if success:
                # Dibujar ejes
                cv2.drawFrameAxes(frame, cam_K, dist_coeffs, rvec, tvec, 0.05) # Ejes de 5cm

                # Matriz T_cam_ref
                rmat, _ = cv2.Rodrigues(rvec)
                T_cam_ref = np.eye(4)
                T_cam_ref[:3, :3] = rmat
                T_cam_ref[:3, 3] = tvec.squeeze()

                # --- CÁLCULO DE TF ---
                # Queremos: T_base_cam (Dónde está la cámara respecto a la base)
                # T_base_cam = T_base_ref * (T_cam_ref)^-1
                
                T_ref_cam = inverse_transform(T_cam_ref)
                T_base_cam = T_base_ref @ T_ref_cam
                
                # Extraer datos
                cam_pos = T_base_cam[:3, 3]
                
                # Convertir rotación a Cuaternión (x, y, z, w) para ROS
                cam_quat = R.from_matrix(T_base_cam[:3, :3]).as_quat() 

                # Mostrar en pantalla
                text_pos = f"POS: X{cam_pos[0]:.3f} Y{cam_pos[1]:.3f} Z{cam_pos[2]:.3f}"
                text_quat= f"QUAT: {cam_quat[0]:.2f} {cam_quat[1]:.2f} {cam_quat[2]:.2f} {cam_quat[3]:.2f}"
                
                cv2.putText(frame, text_pos, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(frame, text_quat, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                
                # Imprimir en terminal (con retorno de carro para no spamear)
                print(f"\r✅ L515 POS: {cam_pos} | QUAT: {cam_quat}", end="")

        cv2.imshow("Validacion L515", frame)
        if cv2.waitKey(1) == ord('q'):
            break

finally:
    pipe.stop()
    cv2.destroyAllWindows()