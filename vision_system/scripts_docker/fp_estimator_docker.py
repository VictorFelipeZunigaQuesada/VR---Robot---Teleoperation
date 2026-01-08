'''
import zmq
import numpy as np
import cv2
import sys
import os
import torch
import trimesh
import nvdiffrast.torch as dr

# ------------------------------------------------------------
# FoundationPose imports
# ------------------------------------------------------------
sys.path.append('/home/felipe/FoundationPose')
from estimater import FoundationPose, ScorePredictor, PoseRefinePredictor
from estimater import draw_posed_3d_box, draw_xyz_axis

# ------------------------------------------------------------
# ZMQ CONFIG
# ------------------------------------------------------------
HOST_IP = "127.0.0.1" 
PORT_IN = "5555"
context = zmq.Context()

# Socket Entrada
print(f"📡 Conectando a ZMQ en {HOST_IP}:{PORT_IN}...")
socket_sub = context.socket(zmq.SUB)
socket_sub.setsockopt(zmq.RCVHWM, 2)
socket_sub.connect(f"tcp://{HOST_IP}:{PORT_IN}")
socket_sub.setsockopt(zmq.SUBSCRIBE, b"POSE_DATA")

# Socket Salida
PORT_OUT = "5556"
socket_pub = context.socket(zmq.PUB)
socket_pub.bind(f"tcp://*:{PORT_OUT}")
print(f"📡 Publicando Poses ZMQ en tcp://*:{PORT_OUT}")

# ------------------------------------------------------------
# MESH CONFIG
# ------------------------------------------------------------
MESH_DIR_BASE = "demo_data/cubo_peligro/model"
MESH_PATH = os.path.join(MESH_DIR_BASE, "model.obj")
MTL_PATH = os.path.join(MESH_DIR_BASE, "model.mtl")

try:
    # --- 🔧 AUTO-REPARACIÓN DEL MATERIAL (MAGIA) ---
    print(f"🔧 Analizando modelo para reparación...")
    
    # 1. Detectar nombre del material dentro del OBJ
    mat_name = "material_0" # Nombre por defecto
    try:
        with open(MESH_PATH, "r") as f:
            for line in f:
                if line.strip().startswith("usemtl"):
                    parts = line.strip().split()
                    if len(parts) > 1:
                        mat_name = parts[1]
                        print(f"   -> Material detectado: {mat_name}")
                        break
    except Exception as e:
        print(f"   Advertencia leyendo OBJ: {e}")

    # 2. Sobreescribir el archivo .mtl con una versión LIMPIA y SEGURA
    # Esto elimina los parámetros raros de Blender (Ke, Ni, etc) que causaban el error.
    safe_mtl_content = f"""newmtl {mat_name}
Ka 1.000 1.000 1.000
Kd 1.000 1.000 1.000
Ks 0.000 0.000 0.000
d 1.0
illum 1
map_Kd texture_map.png
"""
    with open(MTL_PATH, "w") as f:
        f.write(safe_mtl_content)
    print("   -> Archivo .mtl saneado exitosamente.")

    # -----------------------------------------------------------
    # 🚀 CARGA NORMAL (Ahora sí funcionará)
    # -----------------------------------------------------------
    # Quitamos 'skip_materials' porque ahora el material es seguro
    mesh = trimesh.load(MESH_PATH, force='mesh') 
    
    # 🔥 PARCHE DE SEGURIDAD PARA UVs FALTANTES 🔥
    if mesh.visual.uv is None:
        print("⚠️ ALERTA: El .obj no tiene coordenadas UV. Generando unas de emergencia...")
        # Generamos UVs dummy (ceros) para que FoundationPose no explote
        # La textura se verá mal, pero el programa funcionará.
        dummy_uv = np.zeros((len(mesh.vertices), 2), dtype=np.float32)
        
        # Intentamos cargar la textura a la fuerza para asignarla a estos UVs
        tex_path = os.path.join(MESH_DIR_BASE, "texture_map.png")
        if os.path.exists(tex_path):
            from PIL import Image
            im = Image.open(tex_path)
            mat = trimesh.visual.material.SimpleMaterial(image=im)
            mesh.visual = trimesh.visual.TextureVisuals(uv=dummy_uv, material=mat)
        else:
            # Si no hay imagen, solo ponemos los UVs vacíos
            mesh.visual = trimesh.visual.TextureVisuals(uv=dummy_uv)
    mesh.vertices = mesh.vertices.astype(np.float32)
    bbox_3d = np.stack([-mesh.extents / 2, mesh.extents / 2], axis=0).reshape(2, 3)

    pose_estimator = FoundationPose(
        model_pts=mesh.vertices,
        model_normals=mesh.vertex_normals,
        mesh=mesh,
        scorer=ScorePredictor(),
        refiner=PoseRefinePredictor(),
        glctx=dr.RasterizeCudaContext()
    )
    # ... (resto de configuraciones del estimator) ...
    pose_estimator.scorer.cfg['input_resize']  = [160, 160]
    pose_estimator.refiner.cfg['input_resize'] = [160, 160]
    pose_estimator.is_register = False
    pose_estimator.pose_in_obj = None

except Exception as e:
    print(f"❌ Error cargando modelo: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
# ------------------------------------------------------------
# BUCLE PRINCIPAL
# ------------------------------------------------------------
lost_counter = 0
MAX_LOST = 5 
start_tracking = False 

# 🔥 CALIBRACIÓN DE OFFSET (SISTEMÁTICO)
# Detectado experimentalmente: El sistema tiene un bias de +20cm

#Z_BIAS_CORRECTION = 0.10 

try:
    print("Esperando imágenes... (Presiona 'S' para iniciar tracking)")
    while True:
        parts = socket_sub.recv_multipart()
        while socket_sub.poll(timeout=0):
            parts = socket_sub.recv_multipart()
            
        shape_rgb = np.frombuffer(parts[1], dtype=np.int32)
        shape_depth = np.frombuffer(parts[2], dtype=np.int32)
        K_original = np.frombuffer(parts[3], dtype=np.float32).reshape((3, 3))
        color = np.frombuffer(parts[4], dtype=np.uint8).reshape(shape_rgb)
        
        depth_data = np.frombuffer(parts[5], dtype=np.uint16).reshape(shape_depth)
        depth = depth_data.astype(np.float32) * 0.00025 # Factor L515

        H, W = color.shape[:2]
        pose_matrix = None
        vis = color.copy()

        # MODO ESPERA
        if not start_tracking:
            cx, cy = W // 2, H // 2
            cv2.rectangle(vis, (cx - 80, cy - 80), (cx + 80, cy + 80), (255, 0, 0), 2)
            cv2.putText(vis, "1. Pon Objeto Aqui", (cx - 100, cy - 90), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            cv2.putText(vis, "2. Presiona 'S'", (cx - 80, cy + 110), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # MODO REGISTRO
        elif not pose_estimator.is_register:
            print("📸 Capturando...")
            mask_raw = np.zeros((H, W), dtype=np.uint8)
            cx, cy = W // 2, H // 2
            cv2.rectangle(mask_raw, (cx - 80, cy - 80), (cx + 80, cy + 80), 255, -1)
            ob_mask = mask_raw > 0
            
            depth_cleaned = depth.copy()
            depth_cleaned[~ob_mask] = 0.0

            pose_matrix = pose_estimator.register(
                K=K_original, rgb=color, depth=depth_cleaned, ob_mask=ob_mask, iteration=2
            )

            if pose_matrix is not None:
                pose_estimator.pose_in_obj = pose_matrix
                pose_estimator.is_register = True
                lost_counter = 0
                print("✅ Tracking Iniciado")
            else:
                print("⚠️ Falló registro")
                start_tracking = False

        # MODO TRACKING
        else:
            try:
                pose_matrix = pose_estimator.track_one(
                    K=K_original, rgb=color, depth=depth, iteration=2
                )
                if pose_matrix is not None:
                    pose_estimator.pose_in_obj = pose_matrix
                    lost_counter = 0
                    
                    # 🔥 APLICAR CALIBRACIÓN 🔥
                    # Restamos el bias para obtener la distancia real
                    #pose_matrix[2, 3] -= Z_BIAS_CORRECTION

                    # Enviar a ROS (Dato corregido)
                    socket_pub.send_multipart([b"POSE_RESULT", pose_matrix.astype(np.float32).tobytes()])
                    
                    # Visualización
                    vis = draw_posed_3d_box(K_original, vis, pose_matrix, bbox_3d)
                    vis = draw_xyz_axis(vis, pose_matrix, scale=0.05, K=K_original)
                    t = pose_matrix[:3, 3]
                    
                    # Mostrar en pantalla
                    cv2.putText(vis,f"X: {t[0]:.2f}m, Y: {t[1]:.2f}m, Z: {t[2]:.2f}m", (10, 60), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                else:
                    lost_counter += 1
                    if lost_counter > MAX_LOST:
                        print("❌ Objeto Perdido")
                        pose_estimator.is_register = False
                        start_tracking = False 
                        lost_counter = 0
            except RuntimeError:
                pose_estimator.is_register = False
                start_tracking = False

        # Debug Depth
        depth_vis = (depth_data / 20).astype(np.uint8)
        depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
        vis[H-120:H, 0:160] = cv2.resize(depth_vis, (160, 120))
        cv2.putText(vis, "Depth Map", (5, H-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

        cv2.imshow("Docker View", vis[..., ::-1])
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            if not start_tracking:
                start_tracking = True
                print("🚀 START")

except KeyboardInterrupt:
    print("Stop.")
finally:
    cv2.destroyAllWindows()
    context.term()

    '''






























import zmq
import numpy as np
import cv2
import sys
import os
import torch
import trimesh
import nvdiffrast.torch as dr

# ------------------------------------------------------------
# FoundationPose imports
# ------------------------------------------------------------
sys.path.append('/home/felipe/FoundationPose')
from estimater import FoundationPose, ScorePredictor, PoseRefinePredictor
from estimater import draw_posed_3d_box, draw_xyz_axis

# ------------------------------------------------------------
# FUNCIONES AUXILIARES (NUEVO)
# ------------------------------------------------------------
def rotation_matrix_to_euler(R):
    """
    Convierte una matriz de rotación 3x3 a ángulos de Euler (Roll, Pitch, Yaw) en grados.
    Convención usada: XYZ (Tait-Bryan)
    """
    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6

    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0

    # Convertir a grados
    return np.degrees([x, y, z])

# ------------------------------------------------------------
# ZMQ CONFIG
# ------------------------------------------------------------
HOST_IP = "127.0.0.1" 
PORT_IN = "5555"
context = zmq.Context()

# Socket Entrada
print(f"📡 Conectando a ZMQ en {HOST_IP}:{PORT_IN}...")
socket_sub = context.socket(zmq.SUB)
socket_sub.setsockopt(zmq.RCVHWM, 2)
socket_sub.connect(f"tcp://{HOST_IP}:{PORT_IN}")
socket_sub.setsockopt(zmq.SUBSCRIBE, b"POSE_DATA")

# Socket Salida
PORT_OUT = "5556"
socket_pub = context.socket(zmq.PUB)
socket_pub.bind(f"tcp://*:{PORT_OUT}")
print(f"📡 Publicando Poses ZMQ en tcp://*:{PORT_OUT}")

# ------------------------------------------------------------
# MESH CONFIG
# ------------------------------------------------------------
MESH_DIR_BASE = "demo_data/cubo_peligro/model"
MESH_PATH = os.path.join(MESH_DIR_BASE, "model2.obj")
MTL_PATH = os.path.join(MESH_DIR_BASE, "model2.mtl")

try:
    # --- 🔧 AUTO-REPARACIÓN DEL MATERIAL (MAGIA) ---
    print(f"🔧 Analizando modelo para reparación...")
    
    mat_name = "material_0" 
    try:
        with open(MESH_PATH, "r") as f:
            for line in f:
                if line.strip().startswith("usemtl"):
                    parts = line.strip().split()
                    if len(parts) > 1:
                        mat_name = parts[1]
                        print(f"   -> Material detectado: {mat_name}")
                        break
    except Exception as e:
        print(f"   Advertencia leyendo OBJ: {e}")

    safe_mtl_content = f"""newmtl {mat_name}
Ka 1.000 1.000 1.000
Kd 1.000 1.000 1.000
Ks 0.000 0.000 0.000
d 1.0
illum 1
map_Kd texture_map2.png
"""
    with open(MTL_PATH, "w") as f:
        f.write(safe_mtl_content)
    print("   -> Archivo .mtl saneado exitosamente.")

    # -----------------------------------------------------------
    # 🚀 CARGA NORMAL
    # -----------------------------------------------------------
    mesh = trimesh.load(MESH_PATH, force='mesh') 
    
    if mesh.visual.uv is None:
        print("⚠️ ALERTA: Generando UVs de emergencia...")
        dummy_uv = np.zeros((len(mesh.vertices), 2), dtype=np.float32)
        tex_path = os.path.join(MESH_DIR_BASE, "texture_map.png")
        if os.path.exists(tex_path):
            from PIL import Image
            im = Image.open(tex_path)
            mat = trimesh.visual.material.SimpleMaterial(image=im)
            mesh.visual = trimesh.visual.TextureVisuals(uv=dummy_uv, material=mat)
        else:
            mesh.visual = trimesh.visual.TextureVisuals(uv=dummy_uv)
            
    mesh.vertices = mesh.vertices.astype(np.float32)
    bbox_3d = np.stack([-mesh.extents / 2, mesh.extents / 2], axis=0).reshape(2, 3)

    pose_estimator = FoundationPose(
        model_pts=mesh.vertices,
        model_normals=mesh.vertex_normals,
        mesh=mesh,
        scorer=ScorePredictor(),
        refiner=PoseRefinePredictor(),
        glctx=dr.RasterizeCudaContext()
    )
    pose_estimator.scorer.cfg['input_resize']  = [160, 160]
    pose_estimator.refiner.cfg['input_resize'] = [160, 160]
    pose_estimator.is_register = False
    pose_estimator.pose_in_obj = None

except Exception as e:
    print(f"❌ Error cargando modelo: {e}")
    sys.exit(1)

# ------------------------------------------------------------
# BUCLE PRINCIPAL
# ------------------------------------------------------------
lost_counter = 0
MAX_LOST = 5 
start_tracking = False 

try:
    print("Esperando imágenes... (Presiona 'S' para iniciar tracking)")
    while True:
        parts = socket_sub.recv_multipart()
        while socket_sub.poll(timeout=0):
            parts = socket_sub.recv_multipart()
            
        shape_rgb = np.frombuffer(parts[1], dtype=np.int32)
        shape_depth = np.frombuffer(parts[2], dtype=np.int32)
        K_original = np.frombuffer(parts[3], dtype=np.float32).reshape((3, 3))
        color = np.frombuffer(parts[4], dtype=np.uint8).reshape(shape_rgb)
        
        depth_data = np.frombuffer(parts[5], dtype=np.uint16).reshape(shape_depth)
        depth = depth_data.astype(np.float32) * 0.00025 # Factor L515

        H, W = color.shape[:2]
        pose_matrix = None
        vis = color.copy()

        # MODO ESPERA
        if not start_tracking:
            cx, cy = W // 2, H // 2
            cv2.rectangle(vis, (cx - 80, cy - 80), (cx + 80, cy + 80), (255, 0, 0), 2)
            cv2.putText(vis, "1. Pon Objeto Aqui", (cx - 100, cy - 90), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            cv2.putText(vis, "2. Presiona 'S'", (cx - 80, cy + 110), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # MODO REGISTRO
        elif not pose_estimator.is_register:
            print("📸 Capturando...")
            mask_raw = np.zeros((H, W), dtype=np.uint8)
            cx, cy = W // 2, H // 2
            cv2.rectangle(mask_raw, (cx - 80, cy - 80), (cx + 80, cy + 80), 255, -1)
            ob_mask = mask_raw > 0
            
            depth_cleaned = depth.copy()
            depth_cleaned[~ob_mask] = 0.0

            pose_matrix = pose_estimator.register(
                K=K_original, rgb=color, depth=depth_cleaned, ob_mask=ob_mask, iteration=2
            )

            if pose_matrix is not None:
                pose_estimator.pose_in_obj = pose_matrix
                pose_estimator.is_register = True
                lost_counter = 0
                print("✅ Tracking Iniciado")
            else:
                print("⚠️ Falló registro")
                start_tracking = False

        # MODO TRACKING
        else:
            try:
                pose_matrix = pose_estimator.track_one(
                    K=K_original, rgb=color, depth=depth, iteration=2
                )
                if pose_matrix is not None:
                    pose_estimator.pose_in_obj = pose_matrix
                    lost_counter = 0
                    
                    # Enviar a ROS
                    socket_pub.send_multipart([b"POSE_RESULT", pose_matrix.astype(np.float32).tobytes()])
                    
                    # Visualización Caja y Ejes
                    vis = draw_posed_3d_box(K_original, vis, pose_matrix, bbox_3d)
                    vis = draw_xyz_axis(vis, pose_matrix, scale=0.05, K=K_original)
                    
                    # --- EXTRAER DATOS PARA DEBUG VISUAL ---
                    # 1. Traslación (X, Y, Z)
                    t = pose_matrix[:3, 3]
                    
                    # 2. Rotación (Euler Angles) 
                    R = pose_matrix[:3, :3]
                    roll, pitch, yaw = rotation_matrix_to_euler(R)

                    # --- DIBUJAR TEXTO EN PANTALLA ---
                    # Bloque Posición (Verde)
                    cv2.putText(vis, f"POS (m)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(vis, f"X: {t[0]:.2f} Y: {t[1]:.2f} Z: {t[2]:.2f}", (10, 60), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    
                    # Bloque Rotación (Amarillo/Naranja)
                    cv2.putText(vis, f"ROT (deg)", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
                    cv2.putText(vis, f"R: {roll:.1f} P: {pitch:.1f} Y: {yaw:.1f}", (10, 130), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)

                else:
                    lost_counter += 1
                    if lost_counter > MAX_LOST:
                        print("❌ Objeto Perdido")
                        pose_estimator.is_register = False
                        start_tracking = False 
                        lost_counter = 0
            except RuntimeError:
                pose_estimator.is_register = False
                start_tracking = False

        # Debug Depth (Miniatura)
        depth_vis = (depth_data / 20).astype(np.uint8)
        depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
        vis[H-120:H, 0:160] = cv2.resize(depth_vis, (160, 120))
        cv2.putText(vis, "Depth Map", (5, H-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

        cv2.imshow("Docker View", vis[..., ::-1])
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            if not start_tracking:
                start_tracking = True
                print("🚀 START")

except KeyboardInterrupt:
    print("Stop.")
finally:
    cv2.destroyAllWindows()
    context.term()