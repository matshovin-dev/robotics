#!/usr/bin/env python3
"""
Roter OBJ-fil rundt en akse
Håndterer vertices (v) og normaler (vn)
roterer med høyrehånd regel

cd ~/vsCode/assets/3d_models/obj/ax18 
python3 ../../../scripts/rotate_obj.py legLong.obj legLong.obj z 90
cd /Users/matsmac/vsCode/Projects/stwMacC


"""

import sys
import math

def rotate_x(x, y, z, angle_deg):
    """Roter rundt X-akse"""
    angle = math.radians(angle_deg)
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    return x, y*cos_a - z*sin_a, y*sin_a + z*cos_a

def rotate_y(x, y, z, angle_deg):
    """Roter rundt Y-akse"""
    angle = math.radians(angle_deg)
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    return x*cos_a + z*sin_a, y, -x*sin_a + z*cos_a

def rotate_z(x, y, z, angle_deg):
    """Roter rundt Z-akse"""
    angle = math.radians(angle_deg)
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    return x*cos_a - y*sin_a, x*sin_a + y*cos_a, z

def main():
    if len(sys.argv) < 5:
        print("Usage: python3 rotate_obj.py input.obj output.obj axis angle")
        print("")
        print("Arguments:")
        print("  input.obj   - Input OBJ file")
        print("  output.obj  - Output OBJ file")
        print("  axis        - Rotation axis: x, y, or z")
        print("  angle       - Rotation angle in degrees")
        print("")
        print("Examples:")
        print("  python3 rotate_obj.py bunn.obj bunn_rot.obj x 90")
        print("  python3 rotate_obj.py top.obj top_rot.obj y -30")
        print("  python3 rotate_obj.py leg.obj leg_rot.obj z 45")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2]
    axis = sys.argv[3].lower()
    
    try:
        angle = float(sys.argv[4])
    except ValueError:
        print(f"ERROR: Angle must be a number, got '{sys.argv[4]}'")
        sys.exit(1)

    if axis not in ['x', 'y', 'z']:
        print(f"ERROR: Axis must be x, y, or z, got '{axis}'")
        sys.exit(1)

    rotate_func = {'x': rotate_x, 'y': rotate_y, 'z': rotate_z}[axis]

    try:
        with open(input_file, 'r') as fin:
            lines = fin.readlines()
    except FileNotFoundError:
        print(f"ERROR: Could not find file '{input_file}'")
        sys.exit(1)
    except Exception as e:
        print(f"ERROR: Could not read file '{input_file}': {e}")
        sys.exit(1)

    vertex_count = 0
    normal_count = 0

    try:
        with open(output_file, 'w') as fout:
            for line in lines:
                if line.startswith('v '):
                    # Vertex
                    parts = line.split()
                    x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                    nx, ny, nz = rotate_func(x, y, z, angle)
                    fout.write(f"v {nx:.6f} {ny:.6f} {nz:.6f}\n")
                    vertex_count += 1
                elif line.startswith('vn '):
                    # Normal
                    parts = line.split()
                    x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                    nx, ny, nz = rotate_func(x, y, z, angle)
                    fout.write(f"vn {nx:.6f} {ny:.6f} {nz:.6f}\n")
                    normal_count += 1
                else:
                    # Alle andre linjer (faces, comments, etc)
                    fout.write(line)
    except Exception as e:
        print(f"ERROR: Could not write file '{output_file}': {e}")
        sys.exit(1)

    print(f"✓ Rotated {input_file} → {output_file}")
    print(f"  {angle}° around {axis.upper()}-axis")
    print(f"  {vertex_count} vertices, {normal_count} normals")

if __name__ == "__main__":
    main()