"""Export per-scale segmentation labels as colored PLY point clouds."""

import argparse
import open3d as o3d
import numpy as np
import os


def random_color_map(labels):
    """
    Generate a color for each label: -1 becomes black and all others are random.
    """
    unique_labels = np.unique(labels)
    color_map = {}
    for lb in unique_labels:
        if lb == -1:
            color_map[lb] = np.array([0, 0, 0])  # Black.
        else:
            # Assign a random display color to each valid segment label.
            color_map[lb] = np.random.rand(3)   # Random color.
    return color_map


def save_labeled_ply(points, labels, save_path):
    """
    Save the label result of one scale as a colored PLY file.
    """
    color_map = random_color_map(labels)
    colors = np.array([color_map[lb] for lb in labels])

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    o3d.io.write_point_cloud(save_path, pcd)
    print(f"Saved: {save_path}")


def load_labels_with_header(label_file):
    """
    Skip the first line (point_num, scale_num) and read the remaining labels.
    """
    with open(label_file, "r") as f:
        lines = f.readlines()

    # Parse the first line.
    header = lines[0].strip().split()
    point_num, scale_num = int(header[0]), int(header[1])
    print(f"Label file header: point count = {point_num}, scale count = {scale_num}")

    # Read the NxS label matrix from the second line onward.
    label_lines = lines[1:]

    # Parse labels.
    labels = [list(map(int, line.strip().split())) for line in label_lines]
    labels = np.array(labels, dtype=int)

    # Validate the matrix shape.
    if labels.shape[0] != point_num:
        raise ValueError(f"Label row count ({labels.shape[0]}) does not match the declared point count ({point_num})!")

    if labels.shape[1] != scale_num:
        raise ValueError(f"Label column count ({labels.shape[1]}) does not match the declared scale count ({scale_num})!")

    return labels


def process(ply_file, label_file, output_dir="output_scales"):
    # Read the base geometry once, then recolor it per scale column.
    # Read the point cloud.
    pcd = o3d.io.read_point_cloud(ply_file)
    points = np.asarray(pcd.points)
    N = points.shape[0]

    # Create the output directory.
    os.makedirs(output_dir, exist_ok=True)

    # Read labels while skipping the header.
    labels = load_labels_with_header(label_file)

    # Validate consistency.
    if labels.shape[0] != N:
        raise ValueError(f"Point-cloud size {N} does not match label row count {labels.shape[0]}!")

    num_scales = labels.shape[1]
    print(f"Point count: {N}, scale count: {num_scales}")

    # Save one point cloud per scale.
    for s in range(num_scales):
        scale_labels = labels[:, s]
        save_name = os.path.join(output_dir, f"scale_{s}.ply")
        save_labeled_ply(points, scale_labels, save_name)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Export per-scale segmentation labels as colored PLY files.")
    parser.add_argument("ply_file", help="Input point cloud.")
    parser.add_argument("label_file", help="NxS label file.")
    parser.add_argument("output_dir", help="Directory for exported PLY files.")
    args = parser.parse_args()

    process(
        ply_file=args.ply_file,
        label_file=args.label_file,
        output_dir=args.output_dir
    )
