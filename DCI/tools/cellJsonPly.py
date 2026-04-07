"""Extract per-cell point subsets from JSON index files and save them as PLY."""

import argparse
import os
import json
import open3d as o3d
import numpy as np

def extract_and_save_leaf_point_clouds(json_folder, ply_path, output_folder):
    os.makedirs(output_folder, exist_ok=True)

    # Load the source cloud once and reuse its point attributes.
    # Read the source point cloud.
    pcd = o3d.io.read_point_cloud(ply_path)
    total_points = len(pcd.points)

    has_color = len(pcd.colors) == total_points
    has_normal = len(pcd.normals) == total_points

    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors) if has_color else None
    normals = np.asarray(pcd.normals) if has_normal else None

    print(f"Loaded point cloud: {total_points} points")
    print(f"  - Color: {'Yes' if has_color else 'No'}")
    print(f"  - Normal: {'Yes' if has_normal else 'No'}")

    # Iterate through all JSON files.
    for filename in os.listdir(json_folder):
        if not filename.endswith(".json"):
            continue
        json_path = os.path.join(json_folder, filename)

        with open(json_path, "r") as f:
            data = json.load(f)
            indices = data.get("support_points", [])

        indices = sorted(set(indices))
        indices = [i for i in indices if 0 <= i < total_points]

        if not indices:
            print(f"Warning: {filename} has no valid indices, skipped.")
            continue

        # Build a sub-cloud using only the referenced point indices.
        # Build the subset point cloud.
        sub_pcd = o3d.geometry.PointCloud()
        sub_pcd.points = o3d.utility.Vector3dVector(points[indices])

        if has_color:
            sub_pcd.colors = o3d.utility.Vector3dVector(colors[indices])
        if has_normal:
            sub_pcd.normals = o3d.utility.Vector3dVector(normals[indices])

        # Build the output path.
        leaf_id = data.get("leaf_node")
        out_path = os.path.join(output_folder, f"leaf_{leaf_id}.ply")

        o3d.io.write_point_cloud(out_path, sub_pcd)
        print(f"Saved: {out_path} ({len(indices)} points)")

def main():
    parser = argparse.ArgumentParser(description="Extract per-cell point subsets from JSON index files.")
    parser.add_argument("json_folder", help="Directory containing leaf JSON files.")
    parser.add_argument("ply_path", help="Path to the source point cloud.")
    parser.add_argument("output_folder", help="Directory for extracted point-cloud subsets.")
    args = parser.parse_args()

    extract_and_save_leaf_point_clouds(args.json_folder, args.ply_path, args.output_folder)


if __name__ == "__main__":
    main()
