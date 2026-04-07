#!/bin/bash

set -u

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LOG_DIR="$SCRIPT_DIR/log"
MAX_JOBS="${MAX_JOBS:-1}"

# Override these paths from the environment to match the local toolchain.
SMOOTH_FEATURE="${SMOOTH_FEATURE:-/path/to/pdpcComputeMultiScaleFeatures}"
SMOOTH_SEG="${SMOOTH_SEG:-/path/to/pdpcSegmentation}"
GENERATE="${GENERATE:-$SCRIPT_DIR/build/module/generate/generate}"

usage() {
    cat <<EOF
Usage: $0 <input .ply file or directory>

Environment variables:
  SMOOTH_FEATURE   Path to the multi-scale feature executable
  SMOOTH_SEG       Path to the segmentation executable
  GENERATE         Path to the DCI generate executable
  MAX_JOBS         Maximum number of parallel jobs (default: 1)

Examples:
  $0 data/example/model.ply
  $0 data/example_directory
EOF
}

check_command() {
    local path="$1"
    local name="$2"
    if [ ! -x "$path" ]; then
        echo "[ERROR] $name is not executable: $path" >&2
        exit 1
    fi
}

wait_for_jobs() {
    local num_jobs
    while true; do
        num_jobs=$(jobs -rp | wc -l)
        if [ "$num_jobs" -lt "$MAX_JOBS" ]; then
            break
        fi
        sleep 1
    done
}

process_one() {
    local plyfile="$1"
    local dir
    dir="$(dirname "$plyfile")"
    local filename
    filename="$(basename "$plyfile")"
    local name="${filename%.*}"
    local absdir
    absdir="$(realpath "$dir")"
    local workdir="$absdir/data_${name}"
    local jsonfile="$absdir/${name}.json"
    local logfile="$LOG_DIR/${name}.log"

    mkdir -p "$LOG_DIR" "$workdir"

    {
        echo "====== Processing $plyfile ======"

        # Step 1: compute multi-scale features.
        "$SMOOTH_FEATURE" -i "$plyfile" -o "$workdir/data_${name}" -v || {
            echo "[ERROR] Feature computation failed for $plyfile"
            return 1
        }

        # Step 2: run segmentation.
        "$SMOOTH_SEG" -i "$plyfile" \
            -s "$workdir/data_${name}_scales.txt" \
            -f "$workdir/data_${name}_features.txt" \
            -o "$workdir/data_${name}" -v || {
            echo "[ERROR] Segmentation failed for $plyfile"
            return 1
        }

        # Step 3: generate a config for the DCI pipeline.
        cat <<EOF > "$jsonfile"
{
    "processLevel": "0",
    "inputPath": {
        "pointcloudsPath": "$(realpath "$plyfile")",
        "componentPath": "$workdir/data_${name}_comp.txt",
        "featuresPath": "$workdir/data_${name}_features.txt",
        "clustersPath": "",
        "segmentsPath": "$workdir/data_${name}_seg.txt",
        "stabilityPath": "empty",
        "variationPath": "$workdir/data_${name}_geometric_variation.txt",
        "scalePath": "$workdir/data_${name}_scales.txt",
        "pointsLod": "empty"
    },
    "filePath": {
        "energyDataTermPath": "_3_componentsEnergy.txt",
        "stabilityValuesPath": "_4-1_stabilityValues.txt",
        "clusterResultPath": "_4-2_clusterResult.txt",
        "normalDeviationsPath": "_4-1_normalDeviation.txt",
        "curvatureDeviationsPath": "_4-1_curvatureDeviation.txt",
        "componentsEnergyPath": "_4-3_componentsEnergy.json",
        "levelComponents": "_4-3_levelComponents.json",
        "componentLevelsNumber": "_4-1_compLevelsNumber.json"
    },
    "parameters": {
        "lamda_normal": 1.0,
        "lamda_curvature": 0.5,
        "clusterNumber": 3,
        "componentThreshold": 200,
        "basicThreshold": 0.6
    },
    "lambda": {
        "lambda_coverage": 1.0,
        "lambda_compact": 1.0,
        "lambda_persistence": 1.0,
        "lambda_level_0": 0.0,
        "lambda_level_1": 0.0,
        "lambda_level_2": 1.0
    }
}
EOF

        # Step 4: run the extended DCI pipeline.
        echo "[INFO] Running GENERATE"
        "$GENERATE" "$jsonfile" || {
            echo "[ERROR] GENERATE failed for $plyfile"
            return 1
        }

        echo "[SUCCESS] Done processing $plyfile"
    } 2>&1 | tee "$logfile"
}

if [ $# -ne 1 ]; then
    usage
    exit 1
fi

INPUT_PATH="$1"

if [ ! -e "$INPUT_PATH" ]; then
    echo "[ERROR] Input path does not exist: $INPUT_PATH" >&2
    exit 1
fi

check_command "$SMOOTH_FEATURE" "SMOOTH_FEATURE"
check_command "$SMOOTH_SEG" "SMOOTH_SEG"
check_command "$GENERATE" "GENERATE"

if [ -f "$INPUT_PATH" ]; then
    case "$INPUT_PATH" in
        *.ply) ;;
        *)
            echo "[ERROR] Input file must be a .ply file: $INPUT_PATH" >&2
            exit 1
            ;;
    esac
    process_one "$(realpath "$INPUT_PATH")"
    exit $?
fi

find "$(realpath "$INPUT_PATH")" -type f -name "*.ply" | while read -r plyfile; do
    wait_for_jobs
    process_one "$plyfile" &
done

wait
echo "[INFO] All processing complete."
