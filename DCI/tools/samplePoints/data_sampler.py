"""Prepare sampled points and an evaluation CSV template for manual inspection."""

import argparse
import open3d as o3d
import numpy as np
import pandas as pd
import os
import sys
import random

# --- Placeholder FPS implementation used when Open3D does not provide one ---
def farthest_point_sample_indices(points, npoint):
    N = points.shape[0]
    sampled_indices = np.zeros(npoint, dtype=int)
    sampled_indices[0] = random.randint(0, N-1)
    distances = np.full(N, np.inf)

    for i in range(1, npoint):
        # Greedily expand the sampled set by maximizing nearest-set distance.
        last_point = points[sampled_indices[i-1]]
        dist = np.sum((points - last_point)**2, axis=1)
        distances = np.minimum(distances, dist)
        sampled_indices[i] = np.argmax(distances)

    return sampled_indices

def prepare_data(pcd_path, n_samples=100, output_dir="sampled_data"):
    """Run sampling and save all required output files."""
    
    # Ensure that the output directory exists.
    os.makedirs(output_dir, exist_ok=True)
    
    print(f"1. Reading point-cloud file: {pcd_path}")
    if not os.path.exists(pcd_path):
        print(f"Error: file not found: {pcd_path}")
        return
        
    try:
        pcd_original = o3d.io.read_point_cloud(pcd_path)
    except Exception as e:
        print(f"Failed to read point cloud: {e}")
        return

    points_count = len(pcd_original.points)
    if points_count == 0:
        print("The point cloud is empty.")
        return

    print(f"Original point count: {points_count}")

    # Sample indices first, then reuse them for both PLY and CSV outputs.
    # --- 2. FPS sampling ---
    points_array = np.asarray(pcd_original.points)
    sampled_indices = farthest_point_sample_indices(points_array, n_samples)
    
    # Keep track of the actual sample count.
    n_samples_actual = len(sampled_indices)
    
    # --- 3. Build the sampled point cloud ---
    pcd_sampled = pcd_original.select_by_index(sampled_indices)
    
    # --- 4. Save the sampled PLY ---
    ply_output_path = os.path.join(output_dir, "sampled_points.ply")
    o3d.io.write_point_cloud(ply_output_path, pcd_sampled)
    print(f"3. Sampled point cloud saved to: {ply_output_path}")

    # --- 5. Create and save the CSV index file ---
    
    # Sampled point coordinates for reference.
    sampled_points_coords = np.asarray(pcd_sampled.points)
    
    # Create the DataFrame.
    # The CSV keeps geometry and blank rating columns in one place.
    df = pd.DataFrame({
        'sampled_point_id': np.arange(n_samples_actual),        # Ordered sampled-point ID (0 to N-1).
        'original_point_index': sampled_indices,                # Index in the original point cloud.
        'x': sampled_points_coords[:, 0],
        'y': sampled_points_coords[:, 1],
        'z': sampled_points_coords[:, 2],
        'rating_1_user_input': np.nan,                          # User fills in this rating column.
        'rating_2_user_input': np.nan                           # User fills in this rating column.
    })
    
    csv_output_path = os.path.join(output_dir, "evaluation_indices.csv")
    df.to_csv(csv_output_path, index=False)
    print(f"4. Evaluation index/template CSV saved to: {csv_output_path}")
    print("   Please fill in the 'rating_1_user_input' and 'rating_2_user_input' columns in the CSV.")

    print("\nData preparation completed.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Prepare sampled points and an evaluation CSV template.")
    parser.add_argument("ply_file", help="Path to the source point cloud.")
    parser.add_argument("output_dir", help="Directory for sampled evaluation data.")
    parser.add_argument("--n-samples", type=int, default=100, help="Number of points to sample.")
    args = parser.parse_args()

    prepare_data(args.ply_file, args.n_samples, args.output_dir)
