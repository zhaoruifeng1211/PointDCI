#!/bin/bash
set -e

# Check argument count
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <file_path>"
    exit 1
fi

# Extract input file path
file_path="$1"

# Extract file name
file_name=$(basename "$file_path" .ply)

# echo "File name: $file_name"

# Extract parent directory
directory=$(dirname "$file_path")

# Debug output for file name and directory
# echo "File name: $file_name"
# echo "Directory containing the file: $directory"
mkdir -p "$directory/$file_name"

../build/pdpcComputeMultiScaleFeatures -i "$file_path" -o "$directory/$file_name/data_$file_name" -v

../build/pdpcSegmentation -i "$file_path" -s "$directory/$file_name/data_$file_name"_scales.txt -f "$directory/$file_name/data_$file_name"_features.txt -o "$directory/$file_name/data_$file_name" -v
