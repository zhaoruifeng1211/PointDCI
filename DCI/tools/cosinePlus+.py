"""Analyze how selected point-wise DCI values change across multiple scales."""

import argparse
import os
import numpy as np
import matplotlib.pyplot as plt
import csv

def read_scalar_file(path):
    with open(path, 'r') as f:
        return np.array([float(line.strip()) for line in f])

def compute_dci_vector(sim_path, stab_path):
    sim = read_scalar_file(sim_path)
    stab = read_scalar_file(stab_path)
    assert len(sim) == len(stab), f"Length mismatch: {len(sim)} vs {len(stab)}"
    # Store each point as a 2D DCI vector: [similarity, stability].
    return np.stack([sim, stab], axis=1)  # shape: [N, 2]

def compute_dci_strength(dci_vec):
    return np.linalg.norm(dci_vec, axis=1)

def extract_point_profiles(dci_m_dict, point_indices):
    profiles = {}
    for pid in point_indices:
        point_profile = {}
        for m, dci in dci_m_dict.items():
            if pid >= len(dci):
                continue
            c, s = dci[pid]
            # strength = np.linalg.norm([c, s])
            # This script uses an inverted mean instead of vector norm as the score.
            strength =1- np.mean([c, s])
            
            point_profile[m] = [c, s, strength]
        profiles[pid] = point_profile
    return profiles

def save_variance_analysis(profiles, m_list, save_dir):
    os.makedirs(save_dir, exist_ok=True)
    
    # === 1. Point-wise DCI strength and variance across m ===
    with open(os.path.join(save_dir, "point_variance_across_m.csv"), "w", newline='') as f:
        writer = csv.writer(f)
        header = ["point_index"] + [f"DCI@{m}" for m in m_list] + ["variance"]
        writer.writerow(header)

        for pid, m_data in profiles.items():
            dci_list = [m_data[m][2] if m in m_data else np.nan for m in m_list]
            var = np.nanvar(dci_list)
            writer.writerow([pid] + dci_list + [var])

    print("[✓] Saved point-wise DCI variance data across m: point_variance_across_m.csv")

    # === 2. Scale-wise DCI strength and variance across points ===
    with open(os.path.join(save_dir, "scale_variance_across_points.csv"), "w", newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["m_value", "point_dci_values", "mean", "variance"])

        for m in m_list:
            values = []
            for pid, m_data in profiles.items():
                if m in m_data:
                    values.append(m_data[m][2])
            if values:
                mean = np.mean(values)
                var = np.var(values)
                writer.writerow([m, values, mean, var])

    print("[✓] Saved scale-wise DCI variance data across points: scale_variance_across_points.csv")

def main():
    parser = argparse.ArgumentParser(description="Analyze selected point-wise DCI values across multiple scales.")
    parser.add_argument("base_dir", help="Directory containing one subfolder per scale count.")
    parser.add_argument("--m-list", default="20,30,50,80,100", help="Comma-separated scale counts.")
    parser.add_argument("--point-indices", default="431787,89469,165502,116665,149178", help="Comma-separated point indices.")
    parser.add_argument("--name", default="pisa_normalTrans", help="Base file prefix for similarity/stability files.")
    args = parser.parse_args()

    base_dir = args.base_dir
    m_list = [int(value) for value in args.m_list.split(",") if value.strip()]
    point_indices = [int(value) for value in args.point_indices.split(",") if value.strip()]
    pisa_name = args.name
    dci_m_dict = {}

    # Read the DCI vector for each m.
    print("[1] Read DCI vectors at each scale")
    for m in m_list:
        folder = os.path.join(base_dir, str(m))
        sim_path = os.path.join(folder, f"{pisa_name}compNumSimilarityValue.txt")
        stab_path = os.path.join(folder, f"{pisa_name}compNumStableValue.txt")

        dci_vec = compute_dci_vector(sim_path, stab_path)
        dci_m_dict[m] = dci_vec

    # Extract profiles for the selected points.
    print("[2] Extract DCI profiles for structure points")
    profiles = extract_point_profiles(dci_m_dict, point_indices)

    # Save the variance analysis.
    print("[3] Export variance-analysis CSV files")
    save_dir = os.path.join(base_dir, "dci_variance_analysis")
    save_variance_analysis(profiles, m_list, save_dir)

if __name__ == "__main__":
    main()
