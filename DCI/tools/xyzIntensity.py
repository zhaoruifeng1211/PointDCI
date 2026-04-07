"""Split XYZ+DCI text data into a PLY cloud, scalar file, and histogram outputs."""

import argparse
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description="Split XYZ+DCI text data into geometry, scalar, and histogram outputs.")
    parser.add_argument("input_txt", help="Input text file formatted as x y z dci sequences.")
    parser.add_argument("output_ply", help="Output point-cloud path.")
    parser.add_argument("output_dci", help="Output scalar DCI text file.")
    parser.add_argument("output_hist", help="Output histogram image path.")
    parser.add_argument("output_hist_txt", help="Output histogram-bin text file.")
    args = parser.parse_args()

    with open(args.input_txt, 'r') as f:
        data = f.read().split()

    data = [float(x) for x in data]

    points = []
    dci_values = []
    for i in range(0, len(data), 4):
        x, y, z, dci = data[i:i + 4]
        points.append([x, y, z])
        dci_values.append(dci)

    points = np.array(points)
    dci_values = np.array(dci_values)

    np.savetxt(args.output_dci, dci_values, fmt="%.6f")

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    o3d.io.write_point_cloud(args.output_ply, pcd)
    print(f"PLY point cloud saved as {args.output_ply}")
    print(f"DCI values saved as {args.output_dci}")

    one_minus_dci = 1 - dci_values
    bin_edges = np.arange(0, np.max(one_minus_dci) + 0.05, 0.05)
    hist, bins = np.histogram(one_minus_dci, bins=bin_edges)

    with open(args.output_hist_txt, 'w') as f:
        f.write("Bin_Start\tBin_End\tFrequency\n")
        for start, end, freq in zip(bins[:-1], bins[1:], hist):
            f.write(f"{start:.2f}\t{end:.2f}\t{freq}\n")
    print(f"Frequency distribution saved as {args.output_hist_txt}")

    plt.figure(figsize=(8, 6))
    plt.hist(one_minus_dci, bins=bin_edges, edgecolor='black')
    plt.xlabel("1 - DCI Value")
    plt.ylabel("Frequency")
    plt.title("Frequency Distribution of 1 - DCI Values")
    plt.grid(True, linestyle='--', alpha=0.5)
    plt.savefig(args.output_hist)
    plt.show()
    print(f"Histogram saved as {args.output_hist}")


if __name__ == "__main__":
    main()
