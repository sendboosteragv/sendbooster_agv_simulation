#!/usr/bin/env python3
"""
Setup default map for AGV Web Control
Copies the simulation map to the web control data directory
"""
import os
import sys
import shutil
import yaml
from pathlib import Path
from PIL import Image

# Add backend to path
sys.path.insert(0, str(Path(__file__).parent / "backend"))

from config import MAPS_DIR, DATA_DIR

# Source map files
SRC_MAP_DIR = Path(__file__).parent.parent / "models" / "sendbooster_world" / "meshes" / "map"
SRC_PGM = SRC_MAP_DIR / "map.pgm"
SRC_YAML = SRC_MAP_DIR / "map.yaml"

def main():
    print("Setting up default map...")

    # Ensure directories exist
    DATA_DIR.mkdir(exist_ok=True)
    MAPS_DIR.mkdir(exist_ok=True)

    if not SRC_PGM.exists() or not SRC_YAML.exists():
        print(f"Error: Source map files not found in {SRC_MAP_DIR}")
        return 1

    # Copy PGM
    dst_pgm = MAPS_DIR / "default.pgm"
    shutil.copy(SRC_PGM, dst_pgm)
    print(f"Copied {SRC_PGM} -> {dst_pgm}")

    # Convert to PNG
    dst_png = MAPS_DIR / "default.png"
    img = Image.open(dst_pgm)
    img.save(dst_png, "PNG")
    print(f"Created {dst_png}")

    # Copy YAML
    dst_yaml = MAPS_DIR / "default.yaml"
    shutil.copy(SRC_YAML, dst_yaml)
    print(f"Copied {SRC_YAML} -> {dst_yaml}")

    # Read YAML for metadata
    with open(SRC_YAML) as f:
        yaml_data = yaml.safe_load(f)

    # Create database entry
    from database import init_db, SessionLocal
    from models.map import Map

    init_db()
    db = SessionLocal()

    # Check if already exists
    existing = db.query(Map).filter(Map.name == "default").first()
    if existing:
        print("Default map already exists in database")
        db.close()
        return 0

    # Create entry
    width, height = img.size
    origin = yaml_data.get("origin", [0, 0, 0])

    map_obj = Map(
        name="default",
        filename="default.png",
        yaml_config=open(SRC_YAML).read(),
        resolution=yaml_data.get("resolution", 0.05),
        origin_x=origin[0],
        origin_y=origin[1],
        origin_yaw=origin[2] if len(origin) > 2 else 0,
        width=width,
        height=height,
        is_active=True
    )

    db.add(map_obj)
    db.commit()
    print(f"Added map to database: {map_obj.name} ({width}x{height})")

    db.close()
    print("Done!")
    return 0

if __name__ == "__main__":
    sys.exit(main())
