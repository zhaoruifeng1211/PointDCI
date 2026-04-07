"""Read point IDs from one file and summarize their DCI intensities from another."""

import argparse

def read_ids_from_file_a(file_a_path):
    ids = []
    with open(file_a_path, 'r') as f:
        for line in f:
            if line.strip() == '':
                continue
            parts = line.strip().split()
            if len(parts) < 4:
                continue
            try:
                # The 4th column is treated as the point ID lookup key.
                ids.append(int(float(parts[3])))  # The 4th column stores the ID.
            except ValueError:
                continue
    return ids

def read_intensities_by_ids(file_b_path, ids):
    intensities = []
    with open(file_b_path, 'r') as f:
        lines = f.readlines()

    total_lines = len(lines)
    for id in ids:
        # Use IDs as line numbers into the scalar-value file.
        if 0 <= id < total_lines:
            try:
                intensity = float(lines[id].strip())
                intensities.append(intensity)
            except ValueError:
                continue  # Skip lines that cannot be converted.
        else:
            print(f"Warning: ID {id} is out of range for file B, which has {total_lines} lines")
    return intensities

def compute_average(intensities):
    if not intensities:
        return 0.0
    return sum(intensities) / len(intensities)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Summarize DCI intensities for IDs listed in another file.")
    parser.add_argument("file_a_path", help="Path to file A containing point IDs in the 4th column.")
    parser.add_argument("file_b_path", help="Path to the scalar DCI value file.")
    args = parser.parse_args()

    ids = read_ids_from_file_a(args.file_a_path)
    intensities = read_intensities_by_ids(args.file_b_path, ids)
    avg_intensity = compute_average(intensities)

    print(f"The average intensity for the IDs referenced by file A is: {avg_intensity:.6f}")
