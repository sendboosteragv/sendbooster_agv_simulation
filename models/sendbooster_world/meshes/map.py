import trimesh
import numpy as np
from PIL import Image
import yaml
import os

def sample_triangle(v0, v1, v2, n_samples):
    """삼각형 내 랜덤 포인트 샘플링 (barycentric coordinates)"""
    u = np.random.rand(n_samples)
    v = np.random.rand(n_samples)
    mask = u + v > 1
    u[mask] = 1 - u[mask]
    v[mask] = 1 - v[mask]
    w = 1 - (u + v)
    samples = u[:, None]*v0 + v[:, None]*v1 + w[:, None]*v2
    return samples

# ---------------------------
# 설정
# ---------------------------
STL_FILE = "./sendbooster_world.stl"
OUTPUT_DIR = "./map"
MAP_NAME = "map"

samples_per_triangle = 200
resolution = 0.05  # m / pixel
z_threshold = 1.0  # 바닥 제외, 1m 이상만

# 고정 실제 환경 크기
fixed_x_range = 64.0   # X = 64 m
fixed_y_range = 110.0  # Y = 110 m

# ---------------------------
# STL 로드
# ---------------------------
mesh = trimesh.load(STL_FILE)
mesh.apply_scale(0.001)  # mm -> m

# X=좌우, Y=전방, Z=위
vertices = mesh.vertices[:, [0, 2, 1]]
faces = mesh.faces

# ---------------------------
# 고정 크기로 스케일링
# ---------------------------
x_min, x_max = vertices[:,0].min(), vertices[:,0].max()
y_min, y_max = vertices[:,1].min(), vertices[:,1].max()
cur_x_range = x_max - x_min
cur_y_range = y_max - y_min

scale_x = fixed_x_range / cur_x_range
scale_y = fixed_y_range / cur_y_range
scale = min(scale_x, scale_y)
vertices *= scale

# ---------------------------
# 삼각형 샘플링
# ---------------------------
points = []
for face in faces:
    v0, v1, v2 = vertices[face[0]], vertices[face[1]], vertices[face[2]]
    tri_points = sample_triangle(v0, v1, v2, samples_per_triangle)
    tri_points = tri_points[tri_points[:,2] > z_threshold]
    if len(tri_points) > 0:
        points.append(tri_points)

points = np.vstack(points)
xy_points = points[:, :2]

# ---------------------------
# 2D 그리드 생성
# ---------------------------
x_min, x_max = xy_points[:,0].min(), xy_points[:,0].max()
y_min, y_max = xy_points[:,1].min(), xy_points[:,1].max()

width = int(np.ceil((x_max - x_min) / resolution))
height = int(np.ceil((y_max - y_min) / resolution))

pgm = np.ones((height, width), dtype=np.uint8) * 255

# 좌표 → 그리드 인덱스 변환 (상하좌우 반전)
for pt in xy_points:
    x_idx = int((x_max - pt[0]) / resolution)  # 좌우 반전
    y_idx = int((y_max - pt[1]) / resolution)  # 상하 반전

    # 클램프
    x_idx = min(max(x_idx, 0), width - 1)
    y_idx = min(max(y_idx, 0), height - 1)

    pgm[y_idx, x_idx] = 0

# ---------------------------
# PGM 저장
# ---------------------------
os.makedirs(OUTPUT_DIR, exist_ok=True)
pgm_path = os.path.join(OUTPUT_DIR, f"{MAP_NAME}.pgm")
Image.fromarray(pgm).save(pgm_path)
print(f"{pgm_path} 저장 완료!")

# ---------------------------
# YAML 저장
# ---------------------------
yaml_path = os.path.join(OUTPUT_DIR, f"{MAP_NAME}.yaml")
map_yaml = {
    "image": f"{MAP_NAME}.pgm",
    "resolution": resolution,
    "origin": [float(x_min), float(y_min), 0.0],
    "negate": 0,
    "occupied_thresh": 0.65,
    "free_thresh": 0.196
}
with open(yaml_path, "w") as f:
    yaml.dump(map_yaml, f, default_flow_style=False)
print(f"{yaml_path} 저장 완료!")
