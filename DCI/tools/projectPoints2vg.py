"""Project points from each VG group onto the plane defined by that group."""

import argparse
import numpy as np
import open3d as o3d


def read_vg_points_and_groups(vg_path):
    """
    Read point coordinates, group plane parameters, and group indices from a VG file.
    Returns:
    - points: np.array shape (N, 3)
    - groups: list of {
        "plane": [a, b, c, d],
        "indices": [id1, id2, ...],
    }
    """
    # Read the flattened point array before jumping to group metadata.
    with open(vg_path, 'r') as f:
        lines = f.readlines()

    i = 0
    total = len(lines)

     # ------------------------
    # Read point coordinates; they may span one or multiple lines.
    # ------------------------
    while i < total and not lines[i].startswith("num_points"):
        i += 1
    num_pts = int(lines[i].split(":")[1])
    i += 1

    coords_needed = num_pts * 3
    collected = []
    while len(collected) < coords_needed:
        parts = lines[i].strip().split()
        collected.extend(parts)
        i += 1

    collected = list(map(float, collected[:coords_needed]))
    points = np.array(collected).reshape(num_pts, 3)


    # Skip the colors and normals blocks.
    while not lines[i].startswith("num_groups"):
        i += 1

    # ------------------------
    # Read groups.
    # ------------------------
    num_groups = int(lines[i].split(":")[1])
    i += 1

    groups = []

    for _ in range(num_groups):

        while not lines[i].startswith("group_type"):
            i += 1
        i += 1  # skip group_type

        # num_group_parameters
        i += 1
        # group_parameters: a b c d
        a, b, c, d = map(float, lines[i].split(":")[1].split())
        i += 1

        # group_label
        i += 1

        # group_color
        i += 1

        # group_num_points
        gnum = int(lines[i].split(":")[1])
        i += 1

        # Group point indices may span multiple lines.
        idx = []
        count = 0
        while count < gnum:
            nums = lines[i].strip().split()
            idx.extend(map(int, nums))
            count += len(nums)
            i += 1

        # num_children
        i += 1

        groups.append({
            "plane": [a, b, c, d],
            "indices": idx
        })

    return points, groups


def project_points_to_plane(points, plane):
    """
    points: (M, 3)
    plane: [a, b, c, d]
    Return the projected points with shape (M, 3).
    """
    a, b, c, d = plane
    n = np.array([a, b, c])
    denom = np.dot(n, n)

    # ax + by + cz + d
    # Signed distance divided by squared normal length.
    dist = (points @ n + d) / denom

    # p - dist * normal
    projected = points - np.outer(dist, n)
    return projected


def process_vg_projection(vg_file, save_file="vg_projected_all.ply"):
    points, groups = read_vg_points_and_groups(vg_file)

    projected_all = []

    for g in groups:
        # Project each group's own support points onto its fitted plane.
        idx = g["indices"]
        plane = g["plane"]

        pts_group = points[idx]
        pts_proj = project_points_to_plane(pts_group, plane)

        projected_all.append(pts_proj)

    merged = np.vstack(projected_all)

    # Export as a PLY point cloud.
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(merged)

    o3d.io.write_point_cloud(save_file, pcd)
    print(f"Exported the projected point cloud for all groups: {save_file}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Project points from each VG group onto its fitted plane.")
    parser.add_argument("vg_file", help="Path to the VG file.")
    parser.add_argument("save_file", help="Path to the output projected point cloud.")
    args = parser.parse_args()

    process_vg_projection(args.vg_file, args.save_file)
