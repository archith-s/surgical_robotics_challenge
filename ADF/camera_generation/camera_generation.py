#!/usr/bin/env python3
"""
generate_camera_yaml.py
-----------------------
Adds a new camera entry to camera.yaml (an AMBF multibody config).

Cameras orbit the phantom suture line on a circle of radius r in the XY plane
at z=-0.05 (same height as cameraL/R). The circle is centred on the phantom
at (0, -0.0364) in CameraFrame local space. Every camera looks inward and
slightly downward toward the phantom at (0, -0.0364, -0.2669).

angle=0   → same side as cameraL  (-X side)
angle=180 → same side as cameraR  (+X side)

        angle=0 / cameraL side  (-X)
                |
  angle=270 ---[phantom]--- angle=90
                |
        angle=180 / cameraR side (+X)

Verification:
  cameraL at (-0.002, 0, -0.05) with (0,0,-1) is a close approximation of
  angle=0 at tiny radius — the 3.6mm Y offset and 13° tilt are negligible
  at r=0.002 but become significant at larger radii, hence the correct model.

Usage:
  python generate_camera_yaml.py --name cam0   --radius 0.05 --angle 0
  python generate_camera_yaml.py --name cam90  --radius 0.05 --angle 90
  python generate_camera_yaml.py --name cam180 --radius 0.05 --angle 180
  python generate_camera_yaml.py --name cam270 --radius 0.05 --angle 270

Arguments:
  --name     Camera key/name
  --radius   Distance from phantom centre in XY plane (metres)
  --angle    Angle around circle (0=cameraL side, 180=cameraR side)
  --monitor  Display monitor index (default: 1)
  --yaml     Path to camera.yaml (default: camera.yaml)
"""

import argparse
import math
import os
import re
import sys


# ---------------------------------------------------------------------------
# Constants derived from phantom.yaml and world_stereo_test.yaml
# ---------------------------------------------------------------------------

Z_CAM    = -0.05              # circle height — same as cameraL/R

# Phantom suture line in CameraFrame local space (computed from phantom.yaml)
TARGET_X =  0.0000
TARGET_Y = -0.0364
TARGET_Z = -0.2669


# ---------------------------------------------------------------------------
# Maths
# ---------------------------------------------------------------------------

def circle_to_camera_pose(radius: float, angle_deg: float):
    """
    Place a camera on a circle of `radius` centred on the phantom at
    (TARGET_X, TARGET_Y) in the XY plane at Z_CAM.

    angle=0 puts the camera on the -X side (same as cameraL convention).
    look_at points from the camera position toward (TARGET_X, TARGET_Y, TARGET_Z).
    up is computed via Gram-Schmidt, using world +Z as reference where possible.
    """
    a = math.radians(angle_deg)

    # Circle centred at (TARGET_X, TARGET_Y), angle=0 on -X side
    cx = TARGET_X + (-radius * math.cos(a))
    cy = TARGET_Y + (-radius * math.sin(a))
    cz = Z_CAM

    # Look-at direction toward phantom target
    dx, dy, dz = TARGET_X - cx, TARGET_Y - cy, TARGET_Z - cz
    length = math.sqrt(dx**2 + dy**2 + dz**2)
    lx, ly, lz = dx/length, dy/length, dz/length

    # Up: world +Z where possible (not parallel to look_at), else world +Y
    if abs(lz) > 0.99:
        wx, wy, wz = 0.0, 1.0, 0.0
    else:
        wx, wy, wz = 0.0, 0.0, 1.0

    rx = ly*wz - lz*wy;  ry = lz*wx - lx*wz;  rz = lx*wy - ly*wx
    rlen = math.sqrt(rx**2 + ry**2 + rz**2)
    rx, ry, rz = rx/rlen, ry/rlen, rz/rlen

    ux = ry*lz - rz*ly;  uy = rz*lx - rx*lz;  uz = rx*ly - ry*lx

    return (cx, cy, cz), (lx, ly, lz), (ux, uy, uz)


def fmt(v: float) -> str:
    v = 0.0 if v == 0.0 else v
    s = f"{v:.4f}".rstrip('0').rstrip('.')
    return s if s not in ('', '-') else '0'


# ---------------------------------------------------------------------------
# YAML block builder
# ---------------------------------------------------------------------------

def build_camera_block(name: str, location, look_at, up, monitor: int) -> str:
    lx, ly, lz = location
    ax, ay, az = look_at
    ux, uy, uz = up
    return (
        f"{name}:\n"
        f"  namespace: cameras/\n"
        f"  name: {name}\n"
        f"  controller: Camera\n"
        f"  reliability: reliable\n"
        f"  location: {{ x: {fmt(lx)}, y: {fmt(ly)}, z: {fmt(lz)} }}\n"
        f"  look at: {{ x: {fmt(ax)}, y: {fmt(ay)}, z: {fmt(az)} }}\n"
        f"  up: {{ x: {fmt(ux)}, y: {fmt(uy)}, z: {fmt(uz)} }}\n"
        f"  clipping plane: *cam_common_clipping_plane\n"
        f"  field view angle: *cam_common_fov\n"
        f"  monitor: {monitor}\n"
        f"  parent: BODY CameraFrame\n"
        f"  publish image: True\n"
        f"  publish image interval: 5\n"
        f"  publish image resolution: *cam_common_pub_img_res\n"
        f"  publish depth: True\n"
        f"  publish depth resolution: *cam_common_pub_depth_res\n"
        f"  multipass: True  # Set to True to enable shadows\n"
        f"  mouse control multipliers: {{ pan: 0.1, rotate: 1.0, scroll: 0.1, arcball: 0.1 }}\n"
    )


# ---------------------------------------------------------------------------
# camera.yaml editing helpers
# ---------------------------------------------------------------------------

def camera_exists(yaml_text: str, name: str) -> bool:
    return bool(re.compile(rf'^{re.escape(name)}\s*:', re.MULTILINE).search(yaml_text))


def add_camera_to_list(yaml_text: str, name: str) -> str:
    m = re.search(r'^(cameras:\s*)\[\s*\]', yaml_text, re.MULTILINE)
    if m:
        return yaml_text[:m.start()] + f"cameras: [{name}]" + yaml_text[m.end():]
    m = re.search(r'^(cameras:\s*\[)([^\]]+)(\])', yaml_text, re.MULTILINE)
    if m:
        return yaml_text[:m.start()] + f"{m.group(1)}{m.group(2)}, {name}{m.group(3)}" + yaml_text[m.end():]
    m = re.search(r'^(cameras:\s*\n(?:(?:  - [^\n]*\n)*))', yaml_text, re.MULTILINE)
    if m:
        return yaml_text[:m.start()] + m.group(1) + f"  - {name}\n" + yaml_text[m.end():]
    return yaml_text + f"\ncameras: [{name}]\n"


def append_camera_block(yaml_text: str, block: str) -> str:
    sentinel = "# -- Cameras added by generate_camera_yaml.py will be appended below --"
    if sentinel in yaml_text:
        idx = yaml_text.index(sentinel) + len(sentinel)
        return yaml_text[:idx] + "\n\n" + block + yaml_text[idx:]
    return yaml_text.rstrip('\n') + "\n\n" + block + "\n"


def update_camera_yaml(yaml_path: str, name: str, location, look_at, up, monitor: int) -> bool:
    with open(yaml_path, 'r') as f:
        text = f.read()
    if camera_exists(text, name):
        print(f"[warn] Camera '{name}' already exists in {yaml_path} — skipping.")
        return False
    block = build_camera_block(name, location, look_at, up, monitor)
    text = add_camera_to_list(text, name)
    text = append_camera_block(text, block)
    with open(yaml_path, 'w') as f:
        f.write(text)
    print(f"[ok]   Camera '{name}' added to {yaml_path}")
    return True


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(
        description="Add a camera orbiting the phantom suture line to camera.yaml.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument('--name',    required=True,             help='Camera key/name')
    p.add_argument('--radius',  required=True, type=float, help='Distance from phantom centre in XY (metres)')
    p.add_argument('--angle',   required=True, type=float, help='Angle around circle (0=cameraL side, 180=cameraR side)')
    p.add_argument('--monitor', default=1,     type=int,   help='Monitor index (default: 1)')
    p.add_argument('--yaml',    default='camera_generation.yaml',     help='Path to camera.yaml (default: camera.yaml)')
    return p.parse_args()


def main():
    args = parse_args()

    if not os.path.isfile(args.yaml):
        print(f"[error] File not found: {args.yaml}", file=sys.stderr)
        sys.exit(1)

    location, look_at, up = circle_to_camera_pose(args.radius, args.angle)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, -look_at[2]))))

    print(f"\nGenerating camera '{args.name}':")
    print(f"  radius={args.radius} m   angle={args.angle} deg")
    print(f"  location : ({fmt(location[0])}, {fmt(location[1])}, {fmt(location[2])})")
    print(f"  look at  : ({fmt(look_at[0])}, {fmt(look_at[1])}, {fmt(look_at[2])})  [{tilt:.1f}° tilt from straight-down]")
    print(f"  up       : ({fmt(up[0])}, {fmt(up[1])}, {fmt(up[2])})")
    print(f"  target   : ({TARGET_X}, {TARGET_Y}, {TARGET_Z})")
    print()

    success = update_camera_yaml(args.yaml, args.name, location, look_at, up, args.monitor)
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()