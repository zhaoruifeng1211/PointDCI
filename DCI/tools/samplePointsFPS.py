"""Sample representative points with a simple farthest-point-sampling routine."""

import argparse
import open3d as o3d
import numpy as np
import random

def farthest_point_sampling(points, n_samples):
    N = points.shape[0]
    sampled_indices = np.zeros(n_samples, dtype=int)
    sampled_indices[0] = random.randint(0, N-1)
    distances = np.full(N, np.inf)

    for i in range(1, n_samples):
        # Greedily pick the point farthest from the current sampled set.
        last_point = points[sampled_indices[i-1]]
        dist = np.sum((points - last_point)**2, axis=1)
        distances = np.minimum(distances, dist)
        sampled_indices[i] = np.argmax(distances)

    return sampled_indices


def main():
    parser = argparse.ArgumentParser(description="Sample representative points with farthest point sampling.")
    parser.add_argument("input_path", help="Path to the input point cloud.")
    parser.add_argument("--n-samples", type=int, default=100, help="Number of points to sample.")
    parser.add_argument("--output-path", default="fps_sample_with_label.ply", help="Path to the output sampled point cloud.")
    parser.add_argument("--index-path", default="fps_sample_indices.txt", help="Path to the exported sampled-index table.")
    parser.add_argument("--no-draw", action="store_true", help="Disable Open3D visualization.")
    args = parser.parse_args()

    # 1. Read the point cloud.
    input_path = args.input_path
    pcd = o3d.io.read_point_cloud(input_path)
    points = np.asarray(pcd.points)
    print(f"Original point count: {len(points)}")

    # 2. Run FPS sampling.
    n_samples = args.n_samples
    sampled_indices = farthest_point_sampling(points, n_samples)
    sampled_points = points[sampled_indices]
    labels = np.arange(n_samples)  # 0 to 99.

    # 3. Create a new point cloud containing only sampled points.
    fps_pcd = o3d.geometry.PointCloud()
    fps_pcd.points = o3d.utility.Vector3dVector(sampled_points)

    # Add the label attribute.
    # Open3D >= 0.16 supports point-cloud attributes.
    fps_pcd.point["label"] = o3d.utility.IntVector(labels.tolist())

    # 4. Color the sampled points red.
    fps_pcd.paint_uniform_color([1.0, 0.0, 0.0])

    # 5. Save the PLY file.
    output_path = args.output_path
    o3d.io.write_point_cloud(output_path, fps_pcd)
    print(f"Saved sampled point cloud with labels: {output_path}")

    # 6. Visualize the result.
    if not args.no_draw:
        o3d.visualization.draw_geometries([fps_pcd], window_name="FPS Sampling with Label")

    # Save a lookup table from sampled labels back to original point indices.
    # 7. Optionally export the mapping from labels to original indices.
    with open(args.index_path, "w") as f:
        f.write("label,original_index,x,y,z\n")
        for i, idx in enumerate(sampled_indices):
            x, y, z = points[idx]
            f.write(f"{i},{idx},{x},{y},{z}\n")
    print(f"The label-to-original-index mapping has been saved to {args.index_path}")

if __name__ == "__main__":
    main()
