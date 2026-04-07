"""Estimate the voxel occupancy ratio of non-black points inside a point cloud."""

import argparse
import numpy as np
import open3d as o3d
import os

def calculate_voxel_ratio_from_ply_auto(ply_filepath, resolution_divisions=512, black_threshold=10):
    """
    Read a point cloud from a PLY file, estimate a dense voxel size from the
    bounding box, remove black points to obtain the structural subset, and
    compute the voxel occupancy ratio.

    Parameters:
    ply_filepath (str): Full path to the PLY file.
    resolution_divisions (int): Number of divisions for the bounding-box diagonal (N_res).
                                Larger values create denser voxels. Typical choices are 512,
                                1024, or 2048.
    black_threshold (int): Threshold for RGB components in the 0-255 range.

    Returns:
    float: Voxel ratio of the extracted cloud (0.0 to 1.0), or None on failure.
    """
    # Treat non-black points as the extracted structural subset.
    print(f"--- Processing file: {ply_filepath} ---")

    # 1. Read PLY point-cloud data.
    try:
        pcd = o3d.io.read_point_cloud(ply_filepath)
        if not pcd.has_points() or not pcd.has_colors():
            print("Error: the point cloud is missing point or color information.")
            return None
            
        all_points = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors)

    except Exception as e:
        print(f"Error while reading or processing the PLY file: {e}")
        return None

    # 2. Separate the original cloud from the extracted structural cloud.
    original_points = all_points
    threshold_float = black_threshold / 255.0
    
    # Keep only non-black points.
    is_extracted = ~((colors[:, 0] < threshold_float) & 
                     (colors[:, 1] < threshold_float) & 
                     (colors[:, 2] < threshold_float))
                     
    extracted_points = original_points[is_extracted]

    N_original = original_points.shape[0]
    N_extracted = extracted_points.shape[0]
    print(f"Total original points: {N_original}")
    print(f"Extracted structural points (non-black): {N_extracted}")

    if N_extracted == 0:
        print("Warning: no structural points were extracted. The voxel ratio is 0.")
        return 0.0

    # 3. Automatically compute a dense voxel size.

    # a. Compute bounding-box min/max coordinates.
    min_coords = np.min(original_points, axis=0)
    max_coords = np.max(original_points, axis=0)
    
    # b. Compute the bounding-box extent (dx, dy, dz).
    bbox_extent = max_coords - min_coords
    
    # c. Compute the bounding-box diagonal length (L_diag).
    # L_diag = sqrt(dx^2 + dy^2 + dz^2)
    L_diag = np.linalg.norm(bbox_extent)
    
    # d. Set voxel_size automatically.
    # Voxel_size = L_diag / N_res. Larger N_res means denser voxels.
    voxel_size = L_diag / resolution_divisions

    print(f"Bounding-box diagonal length: {L_diag:.4f}")
    print(f"Automatically chosen voxel size: {voxel_size:.6f}")
    print(f"--- Voxelizing with this voxel size ---")

    # 4. Compute the voxel occupancy ratio.

    # a. Local helper: compute the set of voxel indices.
    def get_voxel_indices(points, min_coords, size):
        offset_points = points - min_coords
        # Quantize coordinates into voxel-grid indices.
        # Use `np.floor` to compute voxel indices consistently.
        voxel_indices = np.floor(offset_points / size).astype(np.int64)
        return {tuple(idx) for idx in voxel_indices} # Return the set directly.

    # b. Voxelize and count the original cloud.
    unique_original_voxels = get_voxel_indices(original_points, min_coords, voxel_size)
    N_original_voxels = len(unique_original_voxels)

    # c. Voxelize and count the extracted cloud.
    unique_extracted_voxels = get_voxel_indices(extracted_points, min_coords, voxel_size)
    N_extracted_voxels = len(unique_extracted_voxels)
    
    if N_original_voxels == 0:
        print("Error: the original point cloud occupies zero voxels; voxel_size may be too large.")
        return None

    # d. Compute the final ratio.
    voxel_ratio = N_extracted_voxels / N_original_voxels
    
    print(f"Occupied voxels in the original cloud: {N_original_voxels}")
    print(f"Occupied voxels in the extracted cloud: {N_extracted_voxels}")
    print(f"Final voxel ratio: {voxel_ratio:.6f}")
    
    return voxel_ratio

def save_result(ply_filepath, ratio, res_div):
    """
    Save the result to a `*_voxelRate.txt` file next to the PLY point cloud.
    """
    # Save the computed ratio next to the input PLY for quick bookkeeping.
    if ratio is None:
        return
        
    base_name = os.path.splitext(os.path.basename(ply_filepath))[0]
    output_filename = f"{base_name}_voxelRate_Res{res_div}.txt"
    output_filepath = os.path.join(os.path.dirname(ply_filepath), output_filename)
    
    try:
        with open(output_filepath, 'w') as f:
            f.write(f"Voxel Ratio: {ratio}\n")
            f.write(f"Resolution Divisions (N_res): {res_div}\n")
        print(f"Result saved successfully to: {output_filepath}")
    except Exception as e:
        print(f"Error while saving the file: {e}")

def main():
    parser = argparse.ArgumentParser(description="Estimate the voxel occupancy ratio of non-black points.")
    parser.add_argument("ply_file_path", help="Path to the input point cloud.")
    parser.add_argument("--resolution-divisions", type=int, default=1024, help="Number of bounding-box divisions used to derive the voxel size.")
    args = parser.parse_args()

    ratio = calculate_voxel_ratio_from_ply_auto(args.ply_file_path, args.resolution_divisions)
    if ratio is not None:
        save_result(args.ply_file_path, ratio, args.resolution_divisions)


if __name__ == "__main__":
    main()
