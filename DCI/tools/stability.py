"""Estimate per-point stability from variation curves and colorize the result."""

import argparse
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import os

def read_variation_file(path):
    variation = []
    with open(path, 'r') as f:
        for line in f:
            parts = list(map(float, line.strip().split()))
            variation.append(parts[1:])  # Skip the first index column.
    return variation

def get_local_min_count(var, ratio=0.8):
    var = np.array(var)
    if len(var) < 3:
        return 0
    # threshold = ratio * np.mean(var)
    threshold = ratio
    count = 0
    for i in range(1, len(var) - 1):
        # Count strong local minima as a simple instability signal.
        left_diff = var[i - 1] - var[i]
        right_diff = var[i + 1] - var[i]
        if left_diff > threshold and right_diff > threshold:
            count += 1
    return count

def normalize(data):
    data = np.array(data)
    return (data - data.min()) / (data.max() - data.min() + 1e-8)

def get_color(value):
    """
    Map a normalized value in [0, 1] to RGB using six-piece linear interpolation:
    blue -> cyan -> green -> yellow -> orange -> red
    Returns: r, g, b in [0, 1]
    """
    colors = [
        [0, 0, 1],     # Blue.
        [0, 1, 1],     # Cyan.
        [0, 1, 0],     # Green.
        [1, 1, 0],     # Yellow.
        [1, 0.5, 0],   # Orange.
        [1, 0, 0]      # Red.
    ]

    value = max(0.0, min(1.0, value))  # Clamp to [0, 1].
    idx1 = int(value * 5)
    idx2 = min(idx1 + 1, 5)
    frac = value * 5 - idx1

    r = colors[idx1][0] * (1 - frac) + colors[idx2][0] * frac
    g = colors[idx1][1] * (1 - frac) + colors[idx2][1] * frac
    b = colors[idx1][2] * (1 - frac) + colors[idx2][2] * frac

    return [r, g, b]

def save_stability_color_ply(ply_path, out_ply_path, stability):
    pcd = o3d.io.read_point_cloud(ply_path)
    assert len(stability) == len(pcd.points), "Point counts do not match."

    # Lower normalized stability is rendered with warmer colors via 1 - s.
    colors = [get_color(1 - s) for s in stability] # Use 1 - normalized value for coloring.
    pcd.colors = o3d.utility.Vector3dVector(colors)
    o3d.io.write_point_cloud(out_ply_path, pcd)
    print(f"[✓] Stability-colored point cloud saved to: {out_ply_path}")

def save_stability_txt(stability, txt_path):
    with open(txt_path, 'w') as f:
        for s in stability:
            f.write(f"{s:.6f}\n")
    f.close()
    print(f"[✓] Stability values saved to: {txt_path}")

def main():
    parser = argparse.ArgumentParser(description="Estimate per-point stability from variation curves.")
    parser.add_argument("variation_path", help="Path to the variation-value text file.")
    parser.add_argument("ply_path", help="Path to the source point cloud.")
    parser.add_argument("output_ply_path", help="Path to the colored stability point cloud.")
    parser.add_argument("output_txt_path", help="Path to the exported stability scalar file.")
    parser.add_argument("--ratio", type=float, default=0.01, help="Local-minimum threshold ratio.")
    args = parser.parse_args()

    ratio = args.ratio
    variation_path = args.variation_path
    ply_path = args.ply_path
    output_ply_path = args.output_ply_path
    output_txt_path = args.output_txt_path


    print("[1] Read variation data")
    variation = read_variation_file(variation_path)

    print("[2] Count local minima")
    stability_raw = [get_local_min_count(var, ratio) for var in variation]

    print("[3] Normalize stability")
    stability_norm = normalize(stability_raw)

    print("[4] Map colors and save the point cloud")
    save_stability_color_ply(ply_path, output_ply_path, stability_norm)

    print("[5] Save stability values")
    save_stability_txt(stability_norm, output_txt_path)
    
    check_indices = [61948,98403,165502,116665,113415,39202]   # User-defined indices.
    print("[✓] Stability values for selected points:")
    # for idx in check_indices:
    #     if idx < len(stability_norm):
    #         print(f"Point {idx}: stability = {stability_norm[idx]:.6f}, raw = {stability_raw[idx]}")
    #     else:
    #         print(f"[!] Point {idx} is out of range (total points = {len(stability_norm)})")
            
    # with open(output_result_path, 'w', encoding='utf-8') as f:
    # for idx in check_indices:
    #     if idx < len(stability_norm):
    #         line = f"Point {idx}: stability = {stability_norm[idx]:.6f}, raw = {stability_raw[idx]}"
    #         print(line)  # Print to the console.
    #         # f.write(line + '\n')  # Write to the file.
    #     else:
    #         line = f"[!] Point {idx} is out of range (total points = {len(stability_norm)})"
    #         print(line)  # Print to the console.
            # f.write(line + '\n')  # Write to the file.
    # f.close()
    # print(output_result_path)

if __name__ == "__main__":
    main()
