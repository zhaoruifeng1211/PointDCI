"""Transfer VG group colors onto a PLY point cloud using shared point indices."""

import argparse
import numpy as np
import open3d as o3d


def read_vg(vg_path):
    """
    Parse a VG file and return:
    groups: list of {
        "color": [r, g, b],   # float, [0, 1]
        "indices": [int]
    }
    """
    # Only group colors and group point indices are needed for recoloring.
    groups = []

    with open(vg_path, 'r') as f:
        lines = f.readlines()

    i = 0
    total = len(lines)

    # Find the `num_groups` entry.
    while i < total:
        if lines[i].startswith("num_groups"):
            break
        i += 1

    num_groups = int(lines[i].split(":")[1])
    i += 1

    for _ in range(num_groups):

        # group_type
        while not lines[i].startswith("group_type"):
            i += 1
        i += 1  # skip

        # num_group_parameters
        i += 1
        # group_parameters
        i += 1

        # group_label
        i += 1

        # group_color: r g b
        r, g, b = map(float, lines[i].split(":")[1].split())
        i += 1

        # group_num_points
        num_pts = int(lines[i].split(":")[1])
        i += 1

        # indices
        idx_list = []
        collected = 0
        while collected < num_pts:
            parts = lines[i].strip().split()
            idx_list.extend(map(int, parts))
            collected += len(parts)
            i += 1

        # num_children
        i += 1

        groups.append({
            "color": [r, g, b], 
            "indices": idx_list
        })

    return groups


def apply_color_open3d(ply_path, groups, save_path):
    """
    Use Open3D to load a point cloud, color points by group, and save the result.
    """

    # Read the PLY file.
    pcd = o3d.io.read_point_cloud(ply_path)

    # Add a default color array if the source cloud is geometry-only.
    # If the point cloud has no colors, create a default gray color array.
    if not pcd.has_colors():
        print("The point cloud has no color attribute; creating one automatically.")
        num_pts = np.asarray(pcd.points).shape[0]
        default_color = np.ones((num_pts, 3)) * 0.5   # Gray at 0.5 intensity.
        pcd.colors = o3d.utility.Vector3dVector(default_color)

    colors = np.asarray(pcd.colors)

    # Apply colors group by group.
    for g in groups:
        # Each VG group directly overwrites colors for its own points.
        color = np.array(g["color"])  # Already in the 0 to 1 range.
        idx = g["indices"]
        colors[idx] = color

    pcd.colors = o3d.utility.Vector3dVector(colors)

    # Save the recolored point cloud.
    o3d.io.write_point_cloud(save_path, pcd)
    print(f"Saved the recolored point cloud: {save_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Transfer VG group colors onto a point cloud.")
    parser.add_argument("vg_file", help="Path to the VG file.")
    parser.add_argument("ply_file", help="Path to the source point cloud.")
    parser.add_argument("save_file", help="Path to the recolored output point cloud.")
    args = parser.parse_args()

    groups = read_vg(args.vg_file)
    apply_color_open3d(args.ply_file, groups, args.save_file)
