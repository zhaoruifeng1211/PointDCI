"""Aggregate point-wise intensity into VG groups and cluster the group averages."""

import argparse
from sklearn.cluster import KMeans
import numpy as np

def cluster_group_averages(avg_values, n_clusters=3):
    """
    Cluster the average intensity value of each group.
    """
    # KMeans is applied on 1D average-intensity values only.
    # `avg_values` is shaped like [(group_id, avg_intensity), ...].
    avg_intensity_array = np.array([[val] for _, val in avg_values])  # shape: (N, 1)

    # Run clustering.
    kmeans = KMeans(n_clusters=n_clusters, random_state=0, n_init='auto')
    labels = kmeans.fit_predict(avg_intensity_array)

    # Reattach cluster labels to the original group IDs.
    clustered_result = [(group_id, avg_val, label) 
                        for (group_id, avg_val), label in zip(avg_values, labels)]
    return clustered_result

def parse_vg_groups(vg_path):
    groups = []
    with open(vg_path, 'r') as f:
        lines = f.readlines()

    i = 0
    while i < len(lines):
        line = lines[i].strip()
        if line.startswith('group_num_point:'):
            # Read the following lines until all point indices are collected.
            num_pts = int(line.split(':')[1].strip())
            i += 1
            idx_line = []
            while len(idx_line) < num_pts:
                idx_line += list(map(int, lines[i].strip().split()))
                i += 1
            groups.append(idx_line)
        else:
            i += 1
    print(len(groups), "groups found")
    return groups


def load_float_file(path):
    with open(path, 'r') as f:
        return [float(line.strip()) for line in f.readlines()]


def compute_final_intensity(similarity, stability, w1=0.5, w2=0.5):
    return [w1 * s + w2 * t for s, t in zip(similarity, stability)]


def compute_group_average_intensity(groups, final_intensity):
    avg_values = []
    for idx, group in enumerate(groups):
        values = [final_intensity[i] for i in group]
        avg = sum(values) / len(values) if values else 0
        avg_values.append((idx, avg))
    return avg_values


def main():
    parser = argparse.ArgumentParser(description="Aggregate point-wise intensity into VG groups.")
    parser.add_argument("vg_file", help="Path to the VG file.")
    parser.add_argument("sim_file", help="Path to the similarity-value file.")
    parser.add_argument("stab_file", help="Path to the stability-value file.")
    parser.add_argument("--w1", type=float, default=0.5, help="Weight for similarity.")
    parser.add_argument("--w2", type=float, default=0.5, help="Weight for stability.")
    parser.add_argument("--n-clusters", type=int, default=3, help="Number of clusters for group-average intensity.")
    args = parser.parse_args()

    # Step 1
    groups = parse_vg_groups(args.vg_file)

    # Step 2
    similarity = load_float_file(args.sim_file)
    stability = load_float_file(args.stab_file)
    final_intensity = compute_final_intensity(similarity, stability, w1=args.w1, w2=args.w2)

    # Step 3
    result = compute_group_average_intensity(groups, final_intensity)

    # Cluster the averages, for example into 3 groups.
    clustered = cluster_group_averages(result, n_clusters=args.n_clusters)

    # Print clustered results.
    print("\nClustered Results:")
    for group_id, avg_val, cluster_id in clustered:
        print(f"Group {group_id}: Avg Intensity = {avg_val:.6f}, Cluster ID = {cluster_id}")
    
    # Step 4
    for group_id, avg_val in result:
        print(f'Group {group_id}: Average Intensity = {avg_val:.6f}')


if __name__ == "__main__":
    main()
