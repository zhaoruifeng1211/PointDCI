"""Add bounding-box faces as extra groups to a custom VG scene file."""

import argparse
import numpy as np

def parse_flat_line(line, expected_count, group_size=3):
    floats = list(map(float, line.strip().split()))
    if len(floats) != expected_count * group_size:
        raise ValueError(f"Expected {expected_count*group_size} values, got {len(floats)}")
    return [floats[i:i+group_size] for i in range(0, len(floats), group_size)]

def parse_vg_file(path):
    with open(path, 'r') as f:
        lines = [line.strip() for line in f if line.strip()]

    i = 0
    # Read the flattened point block first.
    # Read points.
    assert lines[i].startswith("num_points:")
    num_points = int(lines[i].split(":")[1])
    i += 1
    points = parse_flat_line(lines[i], num_points, 3)
    i += 1

    # Read colors.
    assert lines[i].startswith("num_colors:")
    num_colors = int(lines[i].split(":")[1])
    i += 1
    if num_colors > 0:
        colors = parse_flat_line(lines[i], num_colors, 3)
        i += 1
    else:
        colors = []

    # Read normals.
    assert lines[i].startswith("num_normals:")
    num_normals = int(lines[i].split(":")[1])
    i += 1
    normals = parse_flat_line(lines[i], num_normals, 3)
    i += 1

    # Read groups.
    groups = []
    while i < len(lines):
        if lines[i].startswith("group_type:"):
            group = {}
            group["group_type"] = lines[i]
            i += 1
            group["num_group_parameters"] = lines[i]
            i += 1
            group["group_parameters"] = lines[i]
            i += 1
            group["group_label"] = lines[i]
            i += 1
            group["group_color"] = lines[i]
            i += 1
            group["group_num_point"] = lines[i]
            num_pts = int(group["group_num_point"].split(":")[1])
            i += 1
            indices = []
            while not lines[i].startswith("num_children:"):
                indices.extend(map(int, lines[i].split()))
                i += 1
            group["indices"] = indices
            group["num_children"] = lines[i]
            i += 1
            groups.append(group)
        else:
            i += 1

    return points, colors, normals, groups

def write_flat_line(f, data):
    # Data is shaped like [[x, y, z], [x, y, z], ...].
    flat = []
    for d in data:
        flat.extend(d)
    f.write(" ".join(map(str, flat)) + "\n")

def write_vg_file(path, points, colors, normals, groups):
    with open(path, 'w') as f:
        f.write(f"num_points: {len(points)}\n")
        write_flat_line(f, points)
        f.write(f"num_colors: {len(colors)}\n")
        if len(colors) > 0:
            write_flat_line(f, colors)
        f.write(f"num_normals: {len(normals)}\n")
        write_flat_line(f, normals)
        f.write(f"num_groups: {len(groups)}\n")
        for g in groups:
            f.write(f"{g['group_type']}\n")
            f.write(f"{g['num_group_parameters']}\n")
            f.write(f"{g['group_parameters']}\n")
            f.write(f"{g['group_label']}\n")
            f.write(f"{g['group_color']}\n")
            f.write(f"{g['group_num_point']}\n")
            idxs = g['indices']
            for j in range(0, len(idxs), 10):
                f.write(" ".join(map(str, idxs[j:j+10])) + "\n")
            f.write(f"{g['num_children']}\n")

def bbox_corners(points):
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]
    min_pt = (min(xs), min(ys), min(zs))
    max_pt = (max(xs), max(ys), max(zs))
    return min_pt, max_pt

def bbox_faces(min_pt, max_pt):
    min_x, min_y, min_z = min_pt
    max_x, max_y, max_z = max_pt
    # Return five faces and skip the top face (the max_z plane).
    return {
        'bottom': [(min_x, min_y, min_z), (max_x, min_y, min_z), (max_x, max_y, min_z), (min_x, max_y, min_z)],
        'left':   [(min_x, min_y, min_z), (min_x, max_y, min_z), (min_x, max_y, max_z), (min_x, min_y, max_z)],
        'right':  [(max_x, min_y, min_z), (max_x, max_y, min_z), (max_x, max_y, max_z), (max_x, min_y, max_z)],
        'front':  [(min_x, min_y, min_z), (max_x, min_y, min_z), (max_x, min_y, max_z), (min_x, min_y, max_z)],
        'back':   [(min_x, max_y, min_z), (max_x, max_y, min_z), (max_x, max_y, max_z), (min_x, max_y, max_z)],
    }

def plane_from_points(points):
    points = np.array(points)
    centroid = np.mean(points, axis=0)
    A = points - centroid
    _, _, vh = np.linalg.svd(A)
    normal = vh[-1]
    a, b, c = normal
    d = -np.dot(normal, centroid)
    return a, b, c, d

def add_bbox_groups(points, groups):
    min_pt, max_pt = bbox_corners(points)
    face_dict = bbox_faces(min_pt, max_pt)

    new_points = []
    new_groups = []
    start_idx = len(points)

    for face_name, face_pts in face_dict.items():
        # Each box face is exported as an extra planar group.
        a, b, c, d = plane_from_points(face_pts)
        indices = list(range(start_idx, start_idx + 4))
        start_idx += 4
        new_points.extend(face_pts)
        group = {
            "group_type": "group_type: 0",
            "num_group_parameters": "num_group_parameters: 4",
            "group_parameters": f"group_parameters: {a:.5f} {b:.5f} {c:.5f} {d:.5f}",
            "group_label": f"group_label: bbox_face_{face_name}",
            "group_color": "group_color: 0.3 0.3 0.3",
            "group_num_point": f"group_num_point: 4",
            "indices": indices,
            "num_children": "num_children: 0"
        }
        new_groups.append(group)

    return new_points, new_groups

def process_vg_with_bbox(input_path, output_path):
    points, colors, normals, groups = parse_vg_file(input_path)

    # Generate new face points and append them to the original scene.
    # Add bounding-box face groups.
    new_points, new_groups = add_bbox_groups(points, groups)

    # Merge original and generated points/groups.
    all_points = points + new_points

    # Assign default colors and normals to the generated points.
    default_color = [0.3, 0.3, 0.3]
    default_normal = [0.0, 0.0, 1.0]
    all_colors = colors + [default_color] * len(new_points)
    all_normals = normals + [default_normal] * len(new_points)

    all_groups = groups + new_groups

    write_vg_file(output_path, all_points, all_colors, all_normals, all_groups)
    print(f"[✓] Saved successfully with bbox face groups included: {output_path}")

# Example invocation.
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Add bounding-box faces as additional groups to a VG file.")
    parser.add_argument("input_vg", help="Path to the input VG file.")
    parser.add_argument("output_vg", help="Path to the output VG file.")
    args = parser.parse_args()
    process_vg_with_bbox(args.input_vg, args.output_vg)
