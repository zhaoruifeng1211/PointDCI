"""Filter VG groups by fused point intensity and write a reduced VG file."""

import argparse
import numpy as np

def read_txt_file(file):
    with open(file, 'r') as f:
        return np.array([float(line.strip()) for line in f])

def parse_flat_points_line(line, num_points):
    """Extract `num_points` 3D points from one flat line, using every three floats as one point."""
    floats = list(map(float, line.strip().split()))
    if len(floats) != num_points * 3:
        raise ValueError(f"Expected {num_points*3} values, got {len(floats)}")
    return [floats[i:i+3] for i in range(0, len(floats), 3)]

def parse_vg(vg_path):
    with open(vg_path, 'r') as f:
        lines = [line.strip() for line in f if line.strip()]

    i = 0
    # Parse the file in the same block order used by the VG exporter.
    assert lines[i].startswith("num_points:")
    num_points = int(lines[i].split(":")[1].strip())
    i += 1
    points = parse_flat_points_line(lines[i], num_points)
    i += 1

    # Colors
    assert lines[i].startswith("num_colors:")
    num_colors = int(lines[i].split(":")[1].strip())
    i += 1
    if num_colors > 0:
        colors = parse_flat_points_line(lines[i], num_colors)
        i += 1
    else:
        colors = []

    assert lines[i].startswith("num_normals:")
    num_normals = int(lines[i].split(":")[1].strip())
    i += 1
    normals = parse_flat_points_line(lines[i], num_normals)
    i += 1

    assert lines[i].startswith("num_groups:")
    num_groups = int(lines[i].split(":")[1])
    i += 1

    groups = []
    for _ in range(num_groups):
        assert lines[i].startswith("group_type:")
        i += 1
        assert lines[i].startswith("num_group_parameters:")
        i += 1
        params = list(map(float, lines[i].strip().split()[1:]))
        i += 1
        label = lines[i].split(":")[1].strip()
        i += 1
        color = list(map(float, lines[i].strip().split()[1:]))
        i += 1
        num_pts = int(lines[i].split(":")[1])
        i += 1
        idxs = []
        while not lines[i].startswith("num_children:"):
            idxs += list(map(int, lines[i].strip().split()))
            i += 1
        assert lines[i].startswith("num_children:")
        i += 1
        groups.append({
            "params": params,
            "label": label,
            "color": color,
            "indices": idxs
        })

    return points, colors, normals, groups

def write_vg(path, points, colors, normals, groups):
    with open(path, 'w') as f:
        f.write(f"num_points: {len(points)}\n")
        flat_points = [f"{p[0]} {p[1]} {p[2]}" for p in points]
        f.write(" ".join(flat_points) + "\n")

        f.write(f"num_colors: {len(colors)}\n")
        if len(colors) > 0:
            flat_colors = [f"{c[0]} {c[1]} {c[2]}" for c in colors]
            f.write(" ".join(flat_colors) + "\n")

        f.write(f"num_normals: {len(normals)}\n")
        flat_normals = [f"{n[0]} {n[1]} {n[2]}" for n in normals]
        f.write(" ".join(flat_normals) + "\n")

        f.write(f"num_groups: {len(groups)}\n")
        for g in groups:
            f.write("group_type: 0\n")
            f.write("num_group_parameters: 4\n")
            f.write(f"group_parameters: {' '.join(map(str, g['params']))}\n")
            f.write(f"group_label: {g['label']}\n")
            f.write(f"group_color: {' '.join(map(str, g['color']))}\n")
            f.write(f"group_num_point: {len(g['indices'])}\n")
            f.write(" ".join(map(str, g['indices'])) + "\n")
            f.write("num_children: 0\n")

def filter_groups_by_intensity(points, colors, normals, groups, similarity, stability, alpha=0.5, threshold=0.5):
    assert len(similarity) == len(points)
    assert len(stability) == len(points)

    # Intensity is a weighted combination of similarity and stability.
    intensities = alpha * similarity + (1 - alpha) * stability

    new_groups = []
    used_indices = set()

    for g in groups:
        group_intensity = np.mean([intensities[idx] for idx in g['indices']])
        if group_intensity >= threshold:
            new_groups.append(g)
            used_indices.update(g['indices'])

    used_indices = sorted(list(used_indices))
    old_to_new_idx = {old: new for new, old in enumerate(used_indices)}

    # Rebuild compact point/color/normal arrays after removing unused points.
    # Colors may be empty in some VG files.
    new_points = [points[i] for i in used_indices]
    new_colors = [colors[i] for i in used_indices] if colors else []
    new_normals = [normals[i] for i in used_indices]

    for g in new_groups:
        g['indices'] = [old_to_new_idx[idx] for idx in g['indices']]

    return new_points, new_colors, new_normals, new_groups

def process_vg_with_intensity_filter(vg_path, similarity_path, stability_path, output_path, alpha=0.5, threshold=0.5):
    print("[1] Read the VG file")
    points, colors, normals, groups = parse_vg(vg_path)

    print("[2] Load similarity and stability")
    similarity = read_txt_file(similarity_path)
    stability = read_txt_file(stability_path)

    print("[3] Filter by intensity")
    new_pts, new_colors, new_normals, new_groups = filter_groups_by_intensity(
        points, colors, normals, groups,
        similarity, stability,
        alpha=alpha, threshold=threshold
    )

    print(f"[4] Write the new VG file, keeping {len(new_groups)} groups and {len(new_pts)} points")
    write_vg(output_path, new_pts, new_colors, new_normals, new_groups)
    print(f"[✓] Saved successfully to: {output_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Filter VG groups by fused point intensity.")
    parser.add_argument("vg_path", help="Path to the input VG file.")
    parser.add_argument("similarity_path", help="Path to the similarity-value file.")
    parser.add_argument("stability_path", help="Path to the stability-value file.")
    parser.add_argument("output_path", help="Path to the filtered VG output.")
    parser.add_argument("--alpha", type=float, default=0.5, help="Weight for similarity in the fused score.")
    parser.add_argument("--threshold", type=float, default=0.68, help="Threshold for keeping groups.")
    args = parser.parse_args()

    process_vg_with_intensity_filter(
        vg_path=args.vg_path,
        similarity_path=args.similarity_path,
        stability_path=args.stability_path,
        output_path=args.output_path,
        alpha=args.alpha,
        threshold=args.threshold
    )
