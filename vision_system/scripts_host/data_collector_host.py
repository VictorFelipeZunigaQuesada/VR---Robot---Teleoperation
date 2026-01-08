
import pyrealsense2 as rs
import numpy as np
import cv2
import cv2.aruco as aruco
import os
import time
import shutil
from ultralytics import SAM
import torch

# =====================================================
# ⚙️ CONFIGURACIÓN DE USUARIO
# =====================================================
OUTPUT_DIR = "demo_data/cubo_peligro"
MARKER_SIZE = 0.10   # <--- ¡AJUSTADO A 3CM! (Verifica tu impresión)
MARKER_ID = 0        

# 🎯 OFFSET SAM:
# Ajustar según dónde cae el cubo respecto al marcador
SAM_OFFSET_MARKER = np.array([0.0, 0.1, 0.014]) 

# =====================================================
# 🛠️ PREPARACIÓN DE CARPETAS
# =====================================================
if os.path.exists(OUTPUT_DIR):
    try:
        shutil.rmtree(OUTPUT_DIR)
    except:
        pass # Si falla borrar, seguimos

folders = ["rgb", "depth", "masks", "cam_in_ob"]
for f in folders:
    os.makedirs(os.path.join(OUTPUT_DIR, f), exist_ok=True)

# =====================================================
# 🧠 CARGAR IA
# =====================================================
print("⏳ Cargando SAM...")
device = 'cuda' if torch.cuda.is_available() else 'cpu'
try:
    model = SAM("sam2.1_b.pt") 
except:
    print("⚠️ Usando mobile_sam.pt")
    model = SAM("mobile_sam.pt")

# =====================================================
# 📷 INICIAR REALSENSE
# =====================================================
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

align = rs.align(rs.stream.color)
profile = pipe.start(cfg)

depth_sensor = profile.get_device().first_depth_sensor()
if depth_sensor.supports(rs.option.visual_preset):
    depth_sensor.set_option(rs.option.visual_preset, 5) # Short Range

intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
K = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]])
dist_coeffs = np.zeros(5) # Asumimos distorsión 0 para RealSense alineada
np.savetxt(f"{OUTPUT_DIR}/cam_K.txt", K)

# =====================================================
# 🏷️ ARUCO & FUNCIONES AUXILIARES
# =====================================================
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
aruco_params = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, aruco_params)

# Puntos 3D del marcador (Esquinas en sentido horario empezando arriba-izq)
# El centro del marcador es 0,0,0
half_size = MARKER_SIZE / 2.0
marker_obj_points = np.array([
    [-half_size, half_size, 0],
    [half_size, half_size, 0],
    [half_size, -half_size, 0],
    [-half_size, -half_size, 0]
], dtype=np.float32)

def create_depth_overlay(depth_im):
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_im, alpha=0.03), cv2.COLORMAP_JET)
    return depth_colormap

saved_count = 0
flash_timer = 0

print("\n--- CONTROLES ---")
print("[ESPACIO] : Guardar Foto")
print("[Q]       : Salir")

try:
    while True:
        frames = pipe.wait_for_frames()
        frames = align.process(frames)
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame: continue

        img_depth = np.asanyarray(depth_frame.get_data())
        img_color = np.asanyarray(color_frame.get_data())
        vis = img_color.copy()
        H, W = vis.shape[:2]

        # 1. Detectar ArUco
        corners, ids, _ = detector.detectMarkers(img_color)
        
        mask = None
        valid_data = False
        depth_quality = 0.0

        if ids is not None and MARKER_ID in ids:
            idx = np.where(ids == MARKER_ID)[0][0]
            current_corners = corners[idx][0] # (4, 2)

            # --- SOLUCIÓN AL ERROR DE ESTIMATEPOSE ---
            # Usamos solvePnP explícitamente
            success, rvec, tvec = cv2.solvePnP(marker_obj_points, current_corners, K, dist_coeffs)
            
            if success:
                # Dibujar Ejes
                cv2.drawFrameAxes(vis, K, dist_coeffs, rvec, tvec, 0.05)
                
                # Transformación
                R_mat, _ = cv2.Rodrigues(rvec)
                T_cam_marker = np.eye(4)
                T_cam_marker[:3, :3] = R_mat
                T_cam_marker[:3, 3] = tvec.squeeze()

                # 2. Calcular Punto SAM
                sam_point_3d = T_cam_marker[:3, 3] + T_cam_marker[:3, :3] @ SAM_OFFSET_MARKER
                sam_point_2d = K @ sam_point_3d

                if sam_point_2d[2] > 0:
                    u, v = int(sam_point_2d[0]/sam_point_2d[2]), int(sam_point_2d[1]/sam_point_2d[2])
                    
                    if 0 <= u < W and 0 <= v < H:
                        cv2.drawMarker(vis, (u, v), (0, 255, 255), cv2.MARKER_CROSS, 20, 2)
                        
                        # 3. SAM
                        results = model.predict(img_color, points=[[u, v]], labels=[1], verbose=False)
                        if results[0].masks is not None:
                            mask = (results[0].masks.data[0].cpu().numpy() * 255).astype(np.uint8)
                            
                            masked_depth = img_depth[mask > 0]
                            if len(masked_depth) > 0:
                                valid_pixels = np.count_nonzero(masked_depth)
                                depth_quality = (valid_pixels / len(masked_depth)) * 100
                                valid_data = True

        # --- VISUALIZACIÓN ---
        if mask is not None:
            green_mask = np.zeros_like(vis)
            green_mask[:, :, 1] = mask 
            vis = cv2.addWeighted(vis, 1.0, green_mask, 0.4, 0)

        depth_vis = create_depth_overlay(img_depth)
        vis[H-120:H, 0:160] = cv2.resize(depth_vis, (160, 120))
        cv2.rectangle(vis, (0, H-120), (160, H), (255, 255, 255), 1)

        cv2.putText(vis, f"Saved: {saved_count}", (W-200, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        if valid_data:
            color_q = (0, 255, 0) if depth_quality > 80 else (0, 0, 255)
            cv2.putText(vis, f"Depth Q: {depth_quality:.1f}%", (170, H-20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_q, 2)

        if flash_timer > 0:
            cv2.rectangle(vis, (0,0), (W,H), (0, 255, 0), 10)
            flash_timer -= 1

        cv2.imshow("Capture", vis)
        key = cv2.waitKey(1)

        # --- GUARDAR ---
        if key == ord(' '):
            if valid_data:
                sid = f"{saved_count:06d}"
                cv2.imwrite(f"{OUTPUT_DIR}/rgb/{sid}.png", img_color)
                cv2.imwrite(f"{OUTPUT_DIR}/depth/{sid}.png", img_depth)
                cv2.imwrite(f"{OUTPUT_DIR}/masks/{sid}.png", mask)
                cam_in_ob = np.linalg.inv(T_cam_marker)
                np.savetxt(f"{OUTPUT_DIR}/cam_in_ob/{sid}.txt", cam_in_ob)
                print(f"📸 Frame {sid} saved.")
                saved_count += 1
                flash_timer = 5
            else:
                print("❌ Ignorado (Mala calidad)")

        elif key == ord('q'):
            break

finally:
    pipe.stop()
    cv2.destroyAllWindows()