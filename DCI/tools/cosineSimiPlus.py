"""Plot DCI profiles of selected points across multiple scale counts."""

import argparse
import os
import numpy as np
import matplotlib.pyplot as plt

def read_scalar_file(path):
    with open(path, 'r') as f:
        return np.array([float(line.strip()) for line in f])

def compute_dci_vector(sim_path, stab_path):
    sim = read_scalar_file(sim_path)
    stab = read_scalar_file(stab_path)
    assert len(sim) == len(stab), f"Length mismatch: {len(sim)} vs {len(stab)}"
    # The two DCI components are kept together for later plotting.
    return np.stack([sim, stab], axis=1)  # shape: [N, 2]

def compute_dci_strength(dci_vec):
    return np.linalg.norm(dci_vec, axis=1)  # DCI strength for each point.

def extract_point_profiles(dci_m_dict, point_indices):
    """
    dci_m_dict: {m_val: np.array([N, 2])}
    point_indices: list[int]
    return: dict{point_idx: {m: [c, s, norm]}}
    """
    profiles = {}
    for pid in point_indices:
        point_profile = {}
        for m, dci in dci_m_dict.items():
            if pid >= len(dci):
                continue
            c, s = dci[pid]
            strength = np.linalg.norm([c, s])
            point_profile[m] = [c, s, strength]
        profiles[pid] = point_profile
    return profiles

def plot_profiles(profiles, m_list, save_dir):
    os.makedirs(save_dir, exist_ok=True)
    for pid, m_data in profiles.items():
        # Plot three aligned curves for each selected point.
        m_vals = sorted(m_data.keys())
        strengths = [m_data[m][2] for m in m_vals]
        consistency = [m_data[m][0] for m in m_vals]
        stability = [m_data[m][1] for m in m_vals]

        plt.figure(figsize=(6, 4))
        plt.plot(m_vals, strengths, marker='o', label='DCI strength')
        plt.plot(m_vals, consistency, marker='^', linestyle='--', label='Consistency')
        plt.plot(m_vals, stability, marker='s', linestyle='--', label='Stability')
        plt.xlabel('Number of Scales (m)')
        plt.ylabel('Value')
        plt.title(f'DCI Profile @ Point {pid}')
        plt.legend()
        plt.grid(True)
        save_path = os.path.join(save_dir, f'dci_profile_point_{pid}.png')
        plt.savefig(save_path, dpi=300)
        print(f"[✓] Saved line chart: {save_path}")
        plt.close()

def main():
    parser = argparse.ArgumentParser(description="Plot DCI profiles of selected points across multiple scale counts.")
    parser.add_argument("base_dir", help="Directory containing one subfolder per scale count.")
    parser.add_argument("--m-list", default="20,30,50,80,100", help="Comma-separated scale counts.")
    parser.add_argument("--point-indices", default="61948,98403,113415,165502,116665", help="Comma-separated point indices.")
    parser.add_argument("--name", default="pisa_normalTrans", help="Base file prefix for similarity/stability files.")
    args = parser.parse_args()

    base_dir = args.base_dir
    m_list = [int(value) for value in args.m_list.split(",") if value.strip()]
    point_indices = [int(value) for value in args.point_indices.split(",") if value.strip()]
    pisa_name = args.name
    dci_m_dict = {}

    # Read DCI vectors at each scale.
    print("[1] Read DCI vectors at each scale")
    for m in m_list:
        folder = os.path.join(base_dir, str(m))
        sim_path = os.path.join(folder, f"{pisa_name}compNumSimilarityValue.txt")
        stab_path = os.path.join(folder, f"{pisa_name}compNumStableValue.txt")

        dci_vec = compute_dci_vector(sim_path, stab_path)
        dci_m_dict[m] = dci_vec

    # Extract values for these points across different m.
    print("[2] Extract DCI profiles for representative points")
    profiles = extract_point_profiles(dci_m_dict, point_indices)

    # Visualize the result.
    print("[3] Plot line charts")
    save_dir = os.path.join(base_dir, "dci_point_profiles")
    plot_profiles(profiles, m_list, save_dir)

if __name__ == "__main__":
    main()
