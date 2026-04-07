#!/bin/bash
set -e

if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <file_path> <-range|-pers|-scales>"
    exit 1
fi

file_path="$1"
strategy="$2"
file_name=$(basename "$file_path" .ply)
directory=$(dirname "$file_path")

mkdir -p "$directory/$file_name"

case "$strategy" in
    -range)
        ../build/pdpcPostProcess -i "$file_path" -s "$directory/$file_name/data_$file_name"_seg.txt -c "$directory/$file_name/data_$file_name"_comp.txt -o "$directory/$file_name/$file_name" -col -v -range 1 2 3 4 5 10 15 20 25 30 35 40 45 50
        ;;
    -pers)
        ../build/pdpcPostProcess -i "$file_path" -s "$directory/$file_name/data_$file_name"_seg.txt -c "$directory/$file_name/data_$file_name"_comp.txt -o "$directory/$file_name/$file_name" -col -v -pers 1 2 3 4 5 10 15 20 25 30 35 40 45 50
        ;;
    -scales)
        ../build/pdpcPostProcess -i "$file_path" -s "$directory/$file_name/data_$file_name"_seg.txt -c "$directory/$file_name/data_$file_name"_comp.txt -o "$directory/$file_name/$file_name" -col -v -scales 2 3 4 5 10 15 20 25 30 35 40 45 50
        ;;
    *)
        echo "Unsupported strategy: $strategy"
        exit 1
        ;;
esac
