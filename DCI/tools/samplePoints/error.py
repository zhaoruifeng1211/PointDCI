"""Compare normalized A/B scores against user ratings stored in a CSV file."""

import argparse
import numpy as np
import csv

# --- Helper function: min-max normalization ---
# This normalization scales one list or numpy array into the [0, 1] range.
def min_max_normalize(data):
    """Apply min-max normalization to a list or numpy array and map it to [0, 1]."""
    data = np.array(data, dtype=float)
    min_val = np.min(data)
    max_val = np.max(data)
    
    # Avoid division by zero: if all values are identical, return 0.5.
    if max_val == min_val:
        return np.full_like(data, 0.5) 
    
    return (data - min_val) / (max_val - min_val)

# --- Task 1: process files a and b ---
def process_ab_files(file_a, file_b):
    """
    Read values from files a and b line by line, normalize them, sum them,
    and take the average. Return the final values as a numpy-backed mapping.
    """
    values_a = []
    values_b = []

    # 1. Read file a.
    try:
        with open(file_a, 'r') as f:
            for line in f:
                values_a.append(float(line.strip()))
    except FileNotFoundError:
        print(f"Error: file {file_a} was not found.")
        return None
    except ValueError:
        print(f"Error: file {file_a} contains non-floating-point values.")
        return None
    
    # 2. Read file b.
    try:
        with open(file_b, 'r') as f:
            for line in f:
                values_b.append(float(line.strip()))
    except FileNotFoundError:
        print(f"Error: file {file_b} was not found.")
        return None
    except ValueError:
        print(f"Error: file {file_b} contains non-floating-point values.")
        return None

    # Check whether the two files have the same number of rows.
    if len(values_a) != len(values_b):
        print("Warning: files a and b have different line counts; the shorter length will be used.")
    
    min_len = min(len(values_a), len(values_b))
    values_a = values_a[:min_len]
    values_b = values_b[:min_len]

    # Normalize both scalar sources against one shared global range.
    # 3. Normalize all values from a and b together.
    # Normalization is based on the global value range across both files.
    # If you need per-row normalization instead, the logic must be changed.
    
    # Convert lists to numpy arrays.
    np_a = np.array(values_a)
    np_b = np.array(values_b)
    
    # Find the global min/max and normalize consistently.
    all_data = np.concatenate([np_a, np_b])
    min_val_global = np.min(all_data)
    max_val_global = np.max(all_data)
    
    if max_val_global == min_val_global:
        # If all values are identical, the normalized result becomes 0.5.
        normalized_a = np.full_like(np_a, 0.5)
        normalized_b = np.full_like(np_b, 0.5)
    else:
        # Standard min-max normalization formula.
        normalized_a = (np_a - min_val_global) / (max_val_global - min_val_global)
        normalized_b = (np_b - min_val_global) / (max_val_global - min_val_global)

    # 4. Add and average the normalized values.
    final_ab_values = (normalized_a + normalized_b) / 2
    
    # Store results in a dictionary keyed by row index for later lookup.
    # The index here is the line number starting from 0.
    ab_results = {i: value for i, value in enumerate(final_ab_values)}
    
    return ab_results

# --- Task 2: process the CSV file ---
def process_csv_file(csv_file):
    """
    Read the CSV file, normalize the selected columns, average them,
    and return a list of `(index, final_value)` records.
    """
    
    # 1. Extract the required data.
    csv_data = []  # Store tuples shaped like (index, value6, value7).
    
    try:
        with open(csv_file, 'r', newline='', encoding='utf-8') as f:
            reader = csv.reader(f)
            # Assume the first row is a header and skip it.
            try:
                next(reader) 
            except StopIteration:
                print(f"Error: file {csv_file} may be empty.")
                return None

            for i, row in enumerate(reader):
                # The second column is the index; the sixth and seventh columns are floats.
                try:
                    index = int(row[1]) 
                    val6 = float(row[5])
                    val7 = float(row[6])
                    csv_data.append((index, val6, val7))
                except IndexError:
                    print(f"Warning: row {i+2} in the CSV does not have enough columns and was skipped.")
                except ValueError:
                    print(f"Warning: row {i+2} in the CSV has an invalid index or numeric format and was skipped.")
    except FileNotFoundError:
        print(f"Error: file {csv_file} was not found.")
        return None
        
    if not csv_data:
        print("No usable CSV data was found.")
        return None

    # Use the two rating columns to build one normalized comparison score.
    # 2. Normalize the sixth and seventh columns together.
    # This keeps normalization consistent across both rating columns.
    all_col6_values = [item[1] for item in csv_data]
    all_col7_values = [item[2] for item in csv_data]
    
    np_col6 = np.array(all_col6_values)
    np_col7 = np.array(all_col7_values)
    
    all_values = np.concatenate([np_col6, np_col7])
    min_val_global = np.min(all_values)
    max_val_global = np.max(all_values)
    
    if max_val_global == min_val_global:
        normalized_col6 = np.full_like(np_col6, 0.5)
        normalized_col7 = np.full_like(np_col7, 0.5)
    else:
        normalized_col6 = (np_col6 - min_val_global) / (max_val_global - min_val_global)
        normalized_col7 = (np_col7 - min_val_global) / (max_val_global - min_val_global)

    # 3. Add and average the normalized values.
    final_csv_values = (normalized_col6 + normalized_col7) / 2

    # 4. Pack the results as (index, final_value).
    csv_results = []
    for i, (index, _, _) in enumerate(csv_data):
        csv_results.append({
            'index': index,
            'csv_value': final_csv_values[i]
        })
        
    return csv_results

# --- Task 3: compute the error ---
def calculate_error(ab_results, csv_results):
    """
    Use the CSV index to look up values from the A/B result and compute
    the absolute error. Print or return the error results.
    """
    
    if ab_results is None or csv_results is None:
        print("Cannot compute error because valid data is missing.")
        return

    errors = []
    
    print("\n--- Error Calculation Results ---")
    print("{:<10} {:<15} {:<15} {:<15}".format("Index", "AB_Value", "CSV_Value", "Absolute_Error"))
    print("-" * 55)

    for row in csv_results:
        csv_index = row['index']
        csv_value = row['csv_value']
        
        # Match each CSV record by the original point index.
        # Look up the matching index in `ab_results`.
        # Here the index corresponds to the row number in files a and b, starting from 0.
        if csv_index in ab_results:
            ab_value = ab_results[csv_index]
            absolute_error = abs(ab_value - csv_value)
            
            errors.append({
                'index': csv_index,
                'ab_value': ab_value,
                'csv_value': csv_value,
                'error': absolute_error
            })
            
            print("{:<10} {:<15.6f} {:<15.6f} {:<15.6f}".format(
                csv_index, ab_value, csv_value, absolute_error))
        else:
            print(f"Warning: CSV index {csv_index} does not exist in the A/B results and was skipped.")
            
    if errors:
        average_error = np.mean([e['error'] for e in errors])
        print("-" * 55)
        print(f"Mean absolute error: {average_error:.6f}")
    else:
        print("No error values were computed successfully.")

# --- Main execution block ---
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Compare normalized A/B scores against manual ratings.")
    parser.add_argument("file_a_path", help="Path to the first scalar file.")
    parser.add_argument("file_b_path", help="Path to the second scalar file.")
    parser.add_argument("csv_file_path", help="Path to the CSV file with user ratings.")
    args = parser.parse_args()

    print("--- Task 1: Process A/B Files ---")
    ab_results = process_ab_files(args.file_a_path, args.file_b_path)

    if ab_results is not None:
        print(f"A/B file processing completed with {len(ab_results)} rows.")
    
    print("\n--- Task 2: Process the CSV File ---")
    csv_results = process_csv_file(args.csv_file_path)
    
    if csv_results is not None:
        print(f"CSV processing completed with {len(csv_results)} valid rows.")

    # --- Task 3: error computation ---
    calculate_error(ab_results, csv_results)

# ----------------------------------------------------
# Make sure numpy is installed before running: `pip install numpy`
# ----------------------------------------------------

# ----------------------------------------------------
# Example file structure (create these manually if needed):
# file_a.txt:
# 10.0
# 20.0
# 30.0
# 40.0
# ...
# 
# file_b.txt:
# 15.0
# 25.0
# 35.0
# 45.0
# ...
#
# data.csv: (the second column is Index, the sixth and seventh columns are floats)
# Col1,Index,Col3,Col4,Col5,Value6,Value7,Col8
# data1,0,xxx,yyy,zzz,100.0,200.0,rest
# data2,1,xxx,yyy,zzz,150.0,250.0,rest
# data3,3,xxx,yyy,zzz,120.0,220.0,rest
# ...
# ----------------------------------------------------
