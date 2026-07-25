#!/usr/bin/env python3
"""
camera_generator.py


Generates or appends a camera to camera_generator.yaml, positioned on a sphere
around LOOK_AT (the psm1/psm2 tool-tip midpoint) and looking at that point.
Designed to be loaded as a multibody config in launch.yaml alongside a world yaml.


Each run adds one camera (camera_0, camera_1, ...) to the yaml file.


IMPORTANT: the "sphere" this orbits is NOT built from raw world (X, Y, Z) axes.
The two PSM tools approach the site at a steep angle that is not aligned with
world +Z, so elevation/azimuth are defined relative to a local basis measured
directly from the tools' own poses (S_AXIS, F_AXIS, U_AXIS below), not the
world frame. See those constants for how they were derived.


Usage:
    python camera_generator.py -r 0.002 -t 90 -p 0     # top-down view (along U_AXIS)
    python camera_generator.py -r 0.002 -t 0  -p 0      # side 1 (along +S_AXIS)
    python camera_generator.py -r 0.002 -t 0  -p 180    # side 2 (along -S_AXIS)
    python camera_generator.py -r 0.002 -t 0  -p 90     # back  (along +F_AXIS, distal)
    python camera_generator.py -r 0.002 -t 0  -p -90 --endoscope-view   # front (along -F_AXIS, proximal)
    python camera_generator.py --reset


Angle convention (elevation + azimuth), all relative to the local (S, F, U) basis:
    --theta / -t  : Elevation angle in degrees [0, 90]
                    Cameras are constrained to the upper hemisphere only —
                    no angle can place the camera below the tools' horizontal plane.
        90 = directly along U_AXIS (true top-down, perpendicular to the tool plane)
         0 = level with the tools (side-on, at the equator of the hemisphere)


    --phi / -p    : Azimuthal angle in degrees [-180, 180], in the S/F plane
          0 = +S_AXIS side  (toward psm1's tip)
         90 = +F_AXIS       (distal/"back": further along the direction both tools insert toward)
        180 = -S_AXIS side  (toward psm2's tip)
        -90 = -F_AXIS       (proximal/"front": back toward where the tools — and endoscope — come from)


Internal mapping: standard spherical theta = 90 - elevation
    elevation=90 -> internal theta=0  -> U-axis coefficient = +r (above) ✓
    elevation=0  -> internal theta=90 -> U-axis coefficient = 0  (side)  ✓
"""


import argparse
import math
import os
import re




# ---------------------------------------------------------------------------
# Scene constant — from ROS2: ros2 topic echo /ambf/env/phantom/Phantom/State
# ---------------------------------------------------------------------------


LOOK_AT = (0.0306997597, 0.1867316746, 0.7336420412) #PSM_MIDPOINT (avg of psm1/toolyawlink + psm2/toolyawlink, fully stretched)
#LOOK_AT = (0.0307, 0.1695, 0.6978) #SIMPLE
#LOOK_AT = (0.0043925177, 0.2601964176, 0.742035985) #STRAIGHT
#LOOK_AT = (0.0564242899, 0.2649616003, .7395174503) #COMPLEX

# ---------------------------------------------------------------------------
# Local tool-frame basis — replaces raw world (X, Y, Z).
#
# Derived from live psm1/psm2 toolpitchlink poses (ros2 topic echo
# /ambf/env/psm{1,2}/toolpitchlink/State), fully-stretched pose, same run the
# LOOK_AT toolyawlink positions above came from:
#   psm1 pos=(0.0535033, 0.1922220, 0.7401852) quat(x,y,z,w)=
#       (0.5509309, -0.2764467, -0.7613498, 0.2009946)
#   psm2 pos=(0.0078410, 0.1922092, 0.7401703) quat(x,y,z,w)=
#       (0.7627069, -0.1992556, -0.5487698, 0.2782574)
#
# F_AXIS: shared insertion axis both tools point along (each tool's local +X,
#   i.e. shaft-toward-tip direction, averaged between psm1 and psm2 — confirmed
#   by cross-checking against the toolpitchlink->toolyawlink displacement
#   vector for each arm independently). This is what "top-down" and "back" were
#   WRONGLY assuming was world -Z / -Y — the true insertion angle is steep and
#   dominated by -Y/-Z, not aligned with any single world axis.
# S_AXIS: side-to-side axis, psm1 tip -> psm2 tip (already very close to world
#   +X, which is why the old side cameras were only "slightly" off — their
#   position was fine, only their up-vector was wrong).
# U_AXIS: true vertical relative to how the tools are angled, i.e.
#   cross(F_AXIS, S_AXIS). NOT world +Z.
#
# ENDOSCOPE_GAZE_DIR (retired): F_AXIS turned out to match the endoscope's
# gaze direction (~6 degrees apart) — F_AXIS is now used directly for
# --endoscope-view cameras since it's measured from real tool data instead of
# triangulated from the phantom's old LOOK_AT.
# ---------------------------------------------------------------------------
F_AXIS = (0.00323768, -0.64277410, -0.76604893)
S_AXIS = (0.99999476, 0.00208395, 0.00247785)
U_AXIS = (0.00000371, -0.76605294, 0.64277748)





# ---------------------------------------------------------------------------
# Maths helpers
# ---------------------------------------------------------------------------


def spherical_to_cartesian(r: float, elevation: float, phi_deg: float):
    """
    elevation: user-facing angle in degrees, +90=along U_AXIS, 0=in the S/F plane
    phi_deg:   azimuthal angle in degrees, 0=+S_AXIS, 90=+F_AXIS, 180=-S_AXIS, -90=-F_AXIS
    Returns (dx, dy, dz) world-frame offset from LOOK_AT, i.e. the (S,F,U)
    coefficients projected through the measured tool-frame basis vectors.
    """
    internal_theta = math.radians(90.0 - elevation)
    phi = math.radians(phi_deg)


    c_s = math.sin(internal_theta) * math.cos(phi)
    c_f = math.sin(internal_theta) * math.sin(phi)
    c_u = math.cos(internal_theta)


    dx = r * (c_s*S_AXIS[0] + c_f*F_AXIS[0] + c_u*U_AXIS[0])
    dy = r * (c_s*S_AXIS[1] + c_f*F_AXIS[1] + c_u*U_AXIS[1])
    dz = r * (c_s*S_AXIS[2] + c_f*F_AXIS[2] + c_u*U_AXIS[2])
    return dx, dy, dz




def compute_up_vector(dx: float, dy: float, dz: float):
    """
    Compute an up vector perpendicular to the look-at direction.
    Uses U_AXIS as primary reference (true vertical relative to the measured
    tool-frame basis, not world +Z). Falls back to F_AXIS at poles (i.e. when
    looking straight along U_AXIS, such as the top-down camera).
    """
    length = math.sqrt(dx*dx + dy*dy + dz*dz)
    if length == 0:
        raise ValueError("Radius must be > 0.")


    # look direction = toward LOOK_AT = -normalize(offset)
    lx, ly, lz = -dx / length, -dy / length, -dz / length


    # Primary up reference: U_AXIS (measured tool-frame vertical)
    wx, wy, wz = U_AXIS
    dot = lx*wx + ly*wy + lz*wz
    if abs(dot) > 0.999:
        # Looking straight along U_AXIS (e.g. top-down) — fall back to F_AXIS
        wx, wy, wz = F_AXIS
        dot = lx*wx + ly*wy + lz*wz


    # Gram-Schmidt orthogonalisation
    ux = wx - dot * lx
    uy = wy - dot * ly
    uz = wz - dot * lz


    u_len = math.sqrt(ux*ux + uy*uy + uz*uz)
    return ux / u_len, uy / u_len, uz / u_len




def fmt(v: float) -> str:
    """Format float to 6 d.p., stripping trailing zeros. Avoids '-0'."""
    v = 0.0 if abs(v) < 1e-9 else v
    return f"{v:.6f}".rstrip('0').rstrip('.')




# ---------------------------------------------------------------------------
# YAML building  (manual strings to preserve anchors & aliases)
# ---------------------------------------------------------------------------


HEADER = """\
# Auto-generated by camera_generator.py
# Each run of the script appends one camera to this file.
# Load as a multibody config in launch.yaml alongside your world yaml.
# Cameras orbit the psm1/psm2 tool-tip midpoint: x={lx}, y={ly}, z={lz}
# Axes are the measured tool-frame basis (S_AXIS, F_AXIS, U_AXIS in
# camera_generator.py), NOT raw world (X, Y, Z) — see that file for how they
# were derived from the tools' live poses.
#
# Angle convention (upper hemisphere only — camera always above the tool plane):
#   theta (elevation): 90=top-down (along U_AXIS), 0=side-on (in S/F plane) [0, 90]
#   phi   (azimuth):    0=+S_AXIS side, 90=+F_AXIS (back/distal),
#                        180/-180=-S_AXIS side, -90=-F_AXIS (front/proximal) [-180, 180]


# CAMERA COMMON CONFIG
cam common configs:
  clipping plane: &cam_common_clipping_plane
    near: 0.001
    far: 10.0
  field of view: &cam_common_fov 1.2
  publish image resolution: &cam_common_pub_img_res {{ width: 640, height: 480 }}
  publish depth resolution: &cam_common_pub_depth_res {{ width: 640, height: 480 }}


""".format(lx=LOOK_AT[0], ly=LOOK_AT[1], lz=LOOK_AT[2])




def camera_block(index, x, y, z, ux, uy, uz, radius, theta, phi, look_at=None, label=None):
    name = f"camera_{index}"
    lx, ly, lz = look_at if look_at is not None else LOOK_AT
    label_str = f" ({label})" if label else ""
    return (
        f"# radius={radius}, theta={theta}deg (elevation), phi={phi}deg (azimuth){label_str}\n"
        f"{name}:\n"
        f"  namespace: cameras/\n"
        f"  name: {name}\n"
        f"  location: {{ x: {fmt(x)}, y: {fmt(y)}, z: {fmt(z)} }}\n"
        f"  look at: {{ x: {fmt(lx)}, y: {fmt(ly)}, z: {fmt(lz)} }}\n"
        f"  up: {{ x: {fmt(ux)}, y: {fmt(uy)}, z: {fmt(uz)} }}\n"
        f"  clipping plane: *cam_common_clipping_plane\n"
        f"  field view angle: *cam_common_fov\n"
        f"  monitor: 1\n"
        f"  publish image: True\n"
        f"  publish image interval: 5\n"
        f"  publish image resolution: *cam_common_pub_img_res\n"
        f"  publish depth: True\n"
        f"  visible: True\n"
        f"  publish depth resolution: *cam_common_pub_depth_res\n"
        f"  multipass: True\n"
        f"  mouse control multipliers: {{ pan: 0.1, rotate: 1.0, scroll: 0.1, arcball: 0.1 }}\n"
    )




# ---------------------------------------------------------------------------
# File read / write helpers
# ---------------------------------------------------------------------------


def read_existing(path):
    if not os.path.exists(path):
        return [], ""
    with open(path, "r") as f:
        text = f.read()
    match = re.search(r'^cameras:\s*\[([^\]]*)\]', text, re.MULTILINE)
    cameras = [c.strip() for c in match.group(1).split(',') if c.strip()] if match else []
    return cameras, text




def cameras_line(cameras):
    return "cameras: [{}]".format(", ".join(cameras))




def write_fresh(path, cameras, first_block):
    with open(path, "w") as f:
        f.write(HEADER)
        f.write(cameras_line(cameras) + "\n\n")
        f.write(first_block)




def append_to_existing(path, cameras, text, new_block):
    updated = re.sub(r'^cameras:\s*\[([^\]]*)\]', cameras_line(cameras), text, flags=re.MULTILINE)
    if not updated.endswith("\n"):
        updated += "\n"
    updated += new_block
    with open(path, "w") as f:
        f.write(updated)




# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(
        description="Append a camera orbiting the psm1/psm2 tool-tip midpoint to camera_generator.yaml."
    )
    parser.add_argument("--radius", "-r", type=float, default=0.1,
                        help="Radius from LOOK_AT, the psm1/psm2 tool-tip midpoint (default: 0.1)")
    parser.add_argument("--theta", "-t", type=float, default=90.0,
                        help="Elevation angle [0, 90]: 90=top-down (along U_AXIS), "
                             "0=side-on (in the S/F plane) (default: 90)")
    parser.add_argument("--phi", "-p", type=float, default=0.0,
                        help="Azimuthal angle [-180, 180]: 0=+S_AXIS, 90=+F_AXIS (back), "
                             "180=-S_AXIS, -90=-F_AXIS (front) (default: 0)")
    parser.add_argument("--output", "-o", type=str, default="camera_generator.yaml",
                        help="Output YAML file (default: camera_generator.yaml)")
    parser.add_argument("--reset", action="store_true",
                        help="Delete the output file and start fresh (no camera added).")
    parser.add_argument("--endoscope-view", action="store_true",
                        help="Look along F_AXIS (the tools' shared insertion direction, "
                             "which matches the endoscope's gaze) instead of orbiting "
                             "inward toward LOOK_AT.")
    parser.add_argument("--label", type=str, default=None,
                        help="Optional human-readable tag appended to the generated comment.")


    args = parser.parse_args()


    if args.reset:
        if os.path.exists(args.output):
            os.remove(args.output)
            print(f"Reset: removed '{args.output}'.")
        else:
            print(f"Reset: '{args.output}' did not exist.")
        return


    if not (0.0 <= args.theta <= 90.0):
        parser.error("--theta must be in [0, 90]: 0=side-on (in the S/F plane), 90=top-down (along U_AXIS).")
    if not (-180.0 <= args.phi <= 180.0):
        parser.error("--phi must be in [-180, 180].")
    if args.radius <= 0:
        parser.error("--radius must be > 0.")


    dx, dy, dz = spherical_to_cartesian(args.radius, args.theta, args.phi)


    x = LOOK_AT[0] + dx
    y = LOOK_AT[1] + dy
    z = LOOK_AT[2] + dz


    if args.endoscope_view:
        gx, gy, gz = F_AXIS
        look_at = (x + gx, y + gy, z + gz)
        # up must be perpendicular to the actual gaze direction being used here,
        # not the orbit-inward direction (-dx, -dy, -dz).
        ux, uy, uz = compute_up_vector(-gx, -gy, -gz)
    else:
        look_at = None
        ux, uy, uz = compute_up_vector(dx, dy, dz)


    cameras, text = read_existing(args.output)
    index = len(cameras)
    new_name = f"camera_{index}"
    cameras.append(new_name)


    block = camera_block(index, x, y, z, ux, uy, uz, args.radius, args.theta, args.phi,
                          look_at=look_at, label=args.label)


    if not text:
        write_fresh(args.output, cameras, block)
    else:
        append_to_existing(args.output, cameras, text, block)


    lx, ly, lz = look_at if look_at is not None else LOOK_AT
    print(f"Added '{new_name}' to '{args.output}'")
    print(f"  Position : x={fmt(x)}, y={fmt(y)}, z={fmt(z)}")
    print(f"  Up vector: x={fmt(ux)}, y={fmt(uy)}, z={fmt(uz)}")
    print(f"  Look at  : x={fmt(lx)}, y={fmt(ly)}, z={fmt(lz)}")
    print(f"  Total cameras in file: {len(cameras)}")




if __name__ == "__main__":
    main()
