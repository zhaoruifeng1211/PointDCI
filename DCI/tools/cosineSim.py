"""Compute cosine similarity between full DCI vectors from different scale settings."""

import argparse
import os
import numpy as np
import matplotlib.pyplot as plt
from sklearn.metrics.pairwise import cosine_similarity

def read_scalar_file(path):
    with open(path, 'r') as f:
        return np.array([float(line.strip()) for line in f])

def compute_dci_vector(sim_path, stab_path):
    sim = read_scalar_file(sim_path)
    stab = read_scalar_file(stab_path)
    assert len(sim) == len(stab), f"Length mismatch: {len(sim)} vs {len(stab)}"
    return np.stack([sim, stab], axis=1)  # shape: [N, 2]

def compute_cosine_similarity_matrix_2d(dci_2d_list):
    # Flatten each scale-specific [N, 2] array into one long descriptor.
    # Flatten each DCI array from [N, 2] to [1, 2N].
    flat_list = [dci.reshape(-1) for dci in dci_2d_list]
    matrix = np.stack(flat_list, axis=0)  # shape: [num_m, 2N]
    return cosine_similarity(matrix)

def plot_similarity_heatmap(sim_matrix, m_list, save_path=None):
    fig, ax = plt.subplots(figsize=(6, 5))
    im = ax.imshow(sim_matrix, cmap="viridis", vmin=0, vmax=1)

    ax.set_xticks(range(len(m_list)))
    ax.set_yticks(range(len(m_list)))
    ax.set_xticklabels(m_list)
    ax.set_yticklabels(m_list)
    ax.set_xlabel("DCI @ m")
    ax.set_ylabel("DCI @ m")
    ax.set_title("2D DCI Cosine Similarity Heatmap")

    cbar = plt.colorbar(im)
    cbar.set_label("Cosine Similarity")

    for i in range(len(m_list)):
        for j in range(len(m_list)):
            # Overlay numeric values for quick comparison.
            ax.text(j, i, f"{sim_matrix[i, j]:.2f}", ha="center", va="center",
                    color="white" if sim_matrix[i, j] < 0.5 else "black", fontsize=8)

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=300)
        print(f"[✓] Heatmap saved to: {save_path}")
    plt.show()

def main():
    parser = argparse.ArgumentParser(description="Compute cosine similarity between DCI vectors from multiple scales.")
    parser.add_argument("base_dir", help="Directory containing one subfolder per scale count.")
    parser.add_argument("--m-list", default="20,30,50,80,100", help="Comma-separated scale counts.")
    parser.add_argument("--name", default="pisa_normalTrans", help="Base file prefix for similarity/stability files.")
    args = parser.parse_args()

    base_dir = args.base_dir
    m_list = [int(value) for value in args.m_list.split(",") if value.strip()]
    point_prefix = args.name

    dci_2d_list = []

    print("[1] Read 2D DCI vectors for each m")
    for m in m_list:
        folder = os.path.join(base_dir, str(m))
        sim_path = os.path.join(folder, f"{point_prefix}compNumSimilarityValue.txt")
        stab_path = os.path.join(folder, f"{point_prefix}compNumStableValue.txt")

        dci_2d = compute_dci_vector(sim_path, stab_path)
        dci_2d_list.append(dci_2d)

    print("[2] Compute the cosine-similarity matrix")
    sim_matrix = compute_cosine_similarity_matrix_2d(dci_2d_list)
    print(sim_matrix)

    print("[3] Plot the heatmap")
    save_path = os.path.join(base_dir, "2D_DCI_similarity_heatmap.png")
    plot_similarity_heatmap(sim_matrix, m_list, save_path)

if __name__ == "__main__":
    main()
