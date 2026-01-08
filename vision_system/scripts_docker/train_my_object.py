

import sys
import os
import glob
import cv2
import numpy as np
import yaml
import trimesh
from pathlib import Path

# --- CONFIGURACIÓN ---
DATASET_DIR = "demo_data/cubo_peligro"  # Tu carpeta de datos grabados
CONFIG_FILE = "bundlesdf/config_ycbv.yml"

# 🔥 LA CORRECCIÓN MÁGICA PARA L515 🔥
# L515 raw unit = 0.25mm.
# Para pasar a Metros: raw * 0.00025
# O lo que es lo mismo: raw / 4000.0
DEPTH_SCALE_DIVISOR = 4000.0 

# Limites para limpiar ruido (en metros)
MIN_DEPTH = 0.2
MAX_DEPTH = 1.5 

def run_reconstruction():
    print(f"🚀 INICIANDO RECONSTRUCCIÓN PARA L515")
    print(f"📂 Datos: {DATASET_DIR}")
    print(f"📏 Escala forzada: 1/{DEPTH_SCALE_DIVISOR}")

    # 1. Setup Paths
    sys.path.append('bundlesdf')
    from bundlesdf.run_nerf import run_neural_object_field
    
    rgb_files = sorted(glob.glob(os.path.join(DATASET_DIR, "rgb", "*.png")))
    if not rgb_files:
        print("❌ No se encontraron imágenes RGB.")
        return

    # 2. Cargar Intrínsecos (K)
    k_path = os.path.join(DATASET_DIR, "cam_K.txt")
    K = np.loadtxt(k_path).reshape(3, 3)

    # 3. Cargar Datos
    rgbs = []
    depths = []
    masks = []
    poses = []

    print("⏳ Cargando frames...")
    for rgb_path in rgb_files:
        stem = Path(rgb_path).stem
        
        # RGB
        rgbs.append(cv2.imread(rgb_path)[:, :, ::-1])

        # DEPTH (CORREGIDA)
        d_path = os.path.join(DATASET_DIR, "depth", f"{stem}.png")
        d_raw = cv2.imread(d_path, -1).astype(np.float32)
        d_meters = d_raw / DEPTH_SCALE_DIVISOR  # <--- AQUÍ ESTÁ EL ARREGLO
        
        # Filtro simple para quitar ruido lejano/cercano
        d_meters[d_meters < MIN_DEPTH] = 0
        d_meters[d_meters > MAX_DEPTH] = 0
        depths.append(d_meters)

        # MASK
        m_path = os.path.join(DATASET_DIR, "masks", f"{stem}.png")
        masks.append(cv2.imread(m_path, 0))

        # POSE (cam_in_ob)
        p_path = os.path.join(DATASET_DIR, "cam_in_ob", f"{stem}.txt")
        poses.append(np.loadtxt(p_path).reshape(4, 4))

    # 4. Configuración BundleSDF
    with open(CONFIG_FILE, 'r') as f:
        cfg = yaml.safe_load(f)
    
    # Ajustes para que sea más rápido y robusto
    cfg['truncation'] = 0.005  # 3cm de margen para la superficie (más tolerante)
    cfg['mesh_resolution'] = 0.0015 # 5mm de resolución (bueno para cajas)
    cfg['empty_weight'] = 5.0
    cfg['octree_smallest_voxel_size'] = 0.002 # 2mm
    cfg['octree_raytracing_voxel_size'] = 0.002 # 2mm

    # 5. Ejecutar BundleSDF
    print("🔥 Entrenando (esto tomará unos minutos)...")
    save_dir = os.path.join(DATASET_DIR, "nerf_output")
    
    try:
        mesh = run_neural_object_field(
            cfg, K, rgbs, depths, masks, poses, save_dir=save_dir
        )
        
        if mesh is None:
            print("❌ Falló la generación del mesh.")
            return

        # 6. Guardar Resultado Final
        out_dir = os.path.join(DATASET_DIR, "model")
        os.makedirs(out_dir, exist_ok=True)
        out_path = os.path.join(out_dir, "model.obj")
        
        # Exportar limpiando vértices sueltos
        mesh.export(out_path)
        print(f"\n✅ ¡LISTO! Modelo guardado en:\n {out_path}")
        print("👉 Abre este archivo en MeshLab o F3D para verificar que se vea como una caja sólida.")

    except Exception as e:
        print(f"\n❌ Error crítico durante reconstrucción: {e}")

if __name__ == "__main__":
    run_reconstruction()



