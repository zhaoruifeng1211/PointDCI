"""Collect support-point indices for leaf cells and export them as JSON files."""

import argparse
import os
import yaml
import json

def find_leaf_indices(tree, path=None, leaves=None):
    if path is None:
        path = []
    if leaves is None:
        leaves = []
    for key, value in tree.items():
        if isinstance(value, dict) and value:
            find_leaf_indices(value, path + [key], leaves)
        else:
            leaves.append(key)
    return leaves

def load_tree_and_build_leaf_dict(yaml_path):
    with open(yaml_path, "r") as f:
        tree = yaml.safe_load(f)
    leaf_indices = find_leaf_indices(tree)
    # Use sets to remove duplicate support-point references.
    return {idx: set() for idx in leaf_indices}  # Use a set to remove duplicates.

def collect_support_points(leaf_dict, txt_folder):
    for filename in os.listdir(txt_folder):
        if not filename.endswith(".txt"):
            continue
        try:
            parts = filename.split("_")
            if len(parts) < 3:
                continue
            A = int(parts[0])
            B = int(parts[1])
        except ValueError:
            continue

        for idx in leaf_dict.keys():
            if idx == A or idx == B:
                # Match files to leaf IDs encoded in the filename prefix.
                file_path = os.path.join(txt_folder, filename)
                with open(file_path, "r") as f:
                    lines = f.readlines()[1:]  # Skip the first line.
                    for line in lines:
                        line = line.strip()
                        if line.isdigit():
                            leaf_dict[idx].add(int(line))
    return leaf_dict

def save_leaf_jsons(leaf_dict, output_folder):
    os.makedirs(output_folder, exist_ok=True)
    for idx, points in leaf_dict.items():
        data = {
            "leaf_node": idx,
            "support_points": sorted(list(points))
        }
        filename = f"leaf_{idx}.json"
        path = os.path.join(output_folder, filename)
        with open(path, "w") as f:
            json.dump(data, f, indent=2)

def main():
    parser = argparse.ArgumentParser(description="Collect support-point indices for leaf cells and export them as JSON.")
    parser.add_argument("tree_yaml_path", help="Path to the tree YAML file.")
    parser.add_argument("txt_folder_path", help="Directory containing support-point text files.")
    parser.add_argument("output_json_folder", help="Directory for exported JSON files.")
    args = parser.parse_args()

    leaf_dict = load_tree_and_build_leaf_dict(args.tree_yaml_path)
    leaf_dict = collect_support_points(leaf_dict, args.txt_folder_path)
    save_leaf_jsons(leaf_dict, args.output_json_folder)

    print(f"All leaf-node JSON files have been saved to: {args.output_json_folder}")


if __name__ == "__main__":
    main()
