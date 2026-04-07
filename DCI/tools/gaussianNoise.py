"""Create noisy point-cloud variants by adding Gaussian perturbations to coordinates."""

import argparse
import open3d as o3d
import numpy as np
import os

def add_gaussian_noise(points, std_dev):
    # Noise is sampled independently for x, y, and z.
    noise = np.random.normal(loc=0.0, scale=std_dev, size=points.shape)
    return points + noise

def process_pointcloud(input_ply_path, output_dir, noise_levels):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    print(f"[1] Read source point cloud: {input_ply_path}")
    pcd = o3d.io.read_point_cloud(input_ply_path)
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors) if pcd.has_colors() else None
    normals = np.asarray(pcd.normals) if pcd.has_normals() else None

    for std_dev in noise_levels:
        noisy_points = add_gaussian_noise(points, std_dev)
        noisy_pcd = o3d.geometry.PointCloud()
        noisy_pcd.points = o3d.utility.Vector3dVector(noisy_points)

        if colors is not None:
            noisy_pcd.colors = o3d.utility.Vector3dVector(colors)

        # Re-estimate normals after the geometry has been perturbed.
        noisy_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30))


        filename = f"noisy_std{std_dev:.4f}.ply"
        output_path = os.path.join(output_dir, filename)
        o3d.io.write_point_cloud(output_path, noisy_pcd)
        print(f"[✓] Saved noisy point cloud with std={std_dev:.4f} to: {output_path}")

def main():
    parser = argparse.ArgumentParser(description="Generate noisy point-cloud variants.")
    parser.add_argument("input_ply_path", help="Path to the source point cloud.")
    parser.add_argument("output_dir", help="Directory for noisy output point clouds.")
    parser.add_argument("--noise-levels", default="0.2,0.25,0.3,0.35,0.4,0.45", help="Comma-separated Gaussian standard deviations.")
    args = parser.parse_args()

    noise_levels = [float(value) for value in args.noise_levels.split(",") if value.strip()]

    process_pointcloud(args.input_ply_path, args.output_dir, noise_levels)

if __name__ == "__main__":
    main()
