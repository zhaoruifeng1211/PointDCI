"""Compute, export, and visualize DCI values from similarity and stability files."""

import argparse
import numpy as np
import open3d as o3d
import os
import matplotlib.pyplot as plt
import csv

def export_selected_point_dci_info(sim, stab, intensity, magnitude, angle, point_indices, save_path):
    os.makedirs(os.path.dirname(save_path), exist_ok=True)

    # Normalize the fused intensity so selected points can be compared consistently.
    # === Normalize values ===
    intensity_norm = (intensity - intensity.min()) / (intensity.max() - intensity.min() + 1e-8)
    # magnitude_norm = (magnitude - magnitude.min()) / (magnitude.max() - magnitude.min() + 1e-8)
    # angle_norm = (angle + np.pi) / (2 * np.pi)  # Map to [0, 1].
    # angle_norm = (angle_norm - angle_norm.min()) / (angle_norm.max() - angle_norm.min() + 1e-8)

    with open(save_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            "Point Index",
            "Similarity",
            "Stability",
            "Weighted Norm (0-1)",
            "Magnitude Norm (0-1)",
            "Angle Norm (0-1)"
        ])
        for idx in point_indices:
            if 0 <= idx < len(sim):
                writer.writerow([
                    idx,
                    f"{sim[idx]:.6f}",
                    f"{stab[idx]:.6f}",
                    f"{1-intensity_norm[idx]:.6f}",
                    # f"{1-magnitude_norm[idx]:.6f}",
                    # f"{angle_norm[idx]:.6f}"  # Optionally invert to keep the color gradient consistent.
                ])
            else:
                print(f"[!] Warning: Point index {idx} out of range (0 ~ {len(sim)-1})")
    print(f"[✓] Saved normalized DCI information to: {save_path}")

def compute_polar_components(sim_path, stab_path):
    sim = read_scalar_file(sim_path)
    stab = read_scalar_file(stab_path)
    assert len(sim) == len(stab), "Similarity and stability lengths do not match."

    # Convert Cartesian DCI components into polar features.
    r = np.sqrt(sim**2 + stab**2)              # Magnitude.
    theta = np.arctan2(stab, sim)              # Direction angle (-pi to pi).
    return r, theta

def colorize_by_magnitude(ply_path, out_path, magnitude):
    pcd = o3d.io.read_point_cloud(ply_path)
    assert len(pcd.points) == len(magnitude), "Point count does not match magnitude count."

    colors = intensity_to_rgb(magnitude, cmap_name='jet')
    pcd.colors = o3d.utility.Vector3dVector(colors)
    o3d.io.write_point_cloud(out_path, pcd)
    print(f"[✓] Saved magnitude-colored point cloud to: {out_path}")

def colorize_by_angle(ply_path, out_path, angle):
    pcd = o3d.io.read_point_cloud(ply_path)
    assert len(pcd.points) == len(angle), "Point count does not match angle count."

    # Map angle from (-pi, pi) to [0, 1] before applying the colormap.
    angle_norm = (angle + np.pi) / (2 * np.pi)
    angle_norm=1-angle_norm
    colors = intensity_to_rgb(angle_norm, cmap_name='jet')
    pcd.colors = o3d.utility.Vector3dVector(colors)
    o3d.io.write_point_cloud(out_path, pcd)
    print(f"[✓] Saved angle-colored point cloud to: {out_path}")

def save_polar_to_file(save_dir, r, theta):
    os.makedirs(save_dir, exist_ok=True)
    # np.save(os.path.join(save_dir, "DCI_magnitude.npy"), r)
    # np.save(os.path.join(save_dir, "DCI_angle.npy"), theta)
    # np.savetxt(os.path.join(save_dir, "DCI_polar.txt"), np.stack([r, theta], axis=1), fmt="%.6f", header="magnitude angle")
    # print(f"[✓] Saved polar-coordinate data to {save_dir}")


def read_scalar_file(path):
    with open(path, 'r') as f:
        return np.array([float(line.strip()) for line in f])

def compute_intensity(sim_path, stab_path, alpha=0.5):
    sim = read_scalar_file(sim_path)
    stab = read_scalar_file(stab_path)
    assert len(sim) == len(stab), "Similarity and stability lengths do not match."
    return alpha * sim + (1 - alpha) * stab

def intensity_to_rgb(values, cmap_name='jet'):
    """Normalize scalar intensity values and map them to RGB colors with the jet colormap."""
    # The colormap is applied after min-max normalization.
    norm = (values - values.min()) / (values.max() - values.min() + 1e-8)
    cmap = plt.get_cmap(cmap_name)
    colors = cmap(norm)[:, :3]  # Return RGB only and drop alpha.
    return colors

def colorize_ply(ply_path, out_path, intensities):
    pcd = o3d.io.read_point_cloud(ply_path)
    assert len(pcd.points) == len(intensities), "Point count does not match intensity count."

    colors = intensity_to_rgb(intensities, cmap_name='jet')
    pcd.colors = o3d.utility.Vector3dVector(colors)

    o3d.io.write_point_cloud(out_path, pcd)
    print(f"[✓] Saved colored point cloud to: {out_path}")

def main():
    parser = argparse.ArgumentParser(description="Compute and export DCI values from similarity and stability files.")
    parser.add_argument("sim_path", help="Path to the similarity-value text file.")
    parser.add_argument("stab_path", help="Path to the stability-value text file.")
    parser.add_argument("ply_path", help="Path to the source point cloud.")
    parser.add_argument("out_path", help="Path to the colored DCI output point cloud.")
    parser.add_argument("--base-out-dir", dest="base_out_dir", default=None, help="Directory for auxiliary outputs. Defaults to the output PLY directory.")
    parser.add_argument("--alpha", type=float, default=1.0, help="Weight for similarity when computing fused DCI.")
    parser.add_argument("--point-indices", default="", help="Comma-separated point indices for CSV export.")
    args = parser.parse_args()

    sim_path = args.sim_path
    stab_path = args.stab_path
    ply_path = args.ply_path
    out_path = args.out_path
    base_out_dir = args.base_out_dir or os.path.dirname(out_path) or "."
    alpha = args.alpha
    point_indices = [int(idx) for idx in args.point_indices.split(",") if idx.strip()]

    print("[1] Load similarity and stability files")
    sim = read_scalar_file(sim_path)
    stab = read_scalar_file(stab_path)
    intensity = compute_intensity(sim_path, stab_path, alpha=alpha)
    np.savetxt(os.path.join(base_out_dir, "DCIValue1.txt"), intensity, fmt='%.8f')
    print("[2] Map colors and save the point cloud")
    colorize_ply(ply_path, out_path, intensity)

    if point_indices:
        print("[3] Export DCI information for selected points")
        export_path = os.path.join(base_out_dir, "selected_points_dci_info.csv")
        export_selected_point_dci_info(sim, stab, intensity, [], [], point_indices, export_path)

if __name__ == "__main__":
    main()
