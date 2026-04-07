"""Evaluate candidate cluster counts for DCI vectors using silhouette and WCSS curves."""

import argparse
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import MiniBatchKMeans
from sklearn.preprocessing import MinMaxScaler
from sklearn.metrics import silhouette_score
import os

def read_and_normalize_dci_data(consistency_file, stability_file):
    """
    Read DCI data from two files, combine them into 2D vectors, and normalize them.
    """
    try:
        # Load data from text files
        consistency_data = np.loadtxt(consistency_file)
        stability_data = np.loadtxt(stability_file)

        if consistency_data.shape != stability_data.shape:
            print("Error: The two data files do not have the same number of points.")
            return None

        # Combine the two scalar files into one 2D feature vector per point.
        # Stack the two components into a 2D DCI vector
        dci_vectors = np.vstack((consistency_data, stability_data)).T
        
        # Normalize the DCI vectors using MinMaxScaler
        scaler = MinMaxScaler()
        normalized_dci = scaler.fit_transform(dci_vectors)
        
        print(f"Data loaded successfully, with {normalized_dci.shape[0]} points in total.")
        return normalized_dci

    except FileNotFoundError:
        print(f"Error: One or both of the files were not found.")
        return None

def find_optimal_k(dci_data, k_range=(2, 10), sample_size=50000):
    """
    Compute silhouette scores and WCSS over a range of K values.
    MiniBatchKMeans and subsampling are used to keep evaluation efficient.
    """
    if dci_data is None:
        return None, None, None
    
    # Subsample large datasets before computing silhouette scores.
    if dci_data.shape[0] > sample_size:
        print(f"Data size ({dci_data.shape[0]}) exceeds {sample_size}; subsampling will be used for silhouette scoring.")
        sample_indices = np.random.choice(dci_data.shape[0], size=sample_size, replace=False)
        dci_sample = dci_data[sample_indices]
    else:
        dci_sample = dci_data

    silhouette_scores = []
    wcss_list = []
    k_values = range(k_range[0], k_range[1] + 1)

    for k in k_values:
        print(f"Processing K = {k}...")
        # MiniBatchKMeans is used here to keep large experiments tractable.
        # Run MiniBatchKMeans for efficient clustering.
        kmeans = MiniBatchKMeans(n_clusters=k, random_state=42, n_init='auto', batch_size=256)
        kmeans.fit(dci_data)
        
        # Compute the silhouette score on the subsample.
        score = silhouette_score(dci_sample, kmeans.predict(dci_sample))
        silhouette_scores.append(score)
        
        # Record WCSS; for MiniBatchKMeans, inertia_ is approximate.
        wcss_list.append(kmeans.inertia_)
        
    return k_values, silhouette_scores, wcss_list

def plot_and_save_results(k_values, silhouette_scores, wcss_list, output_dir='results'):
    """
    Plot and save the silhouette-score and WCSS curves.
    """
    if k_values is None:
        return

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Plot the silhouette-score curve.
    plt.figure(figsize=(12, 5))
    plt.subplot(1, 2, 1)
    plt.plot(k_values, silhouette_scores, marker='o')
    plt.title('Silhouette Score Curve')
    plt.xlabel('Number of Clusters (K)')
    plt.ylabel('Average Silhouette Score')
    plt.grid(True)
    plt.xticks(k_values)
    plt.savefig(os.path.join(output_dir, 'silhouette_score_curve.png'))
    print(f"Silhouette-score curve saved to {output_dir}/silhouette_score_curve.png")

    # Plot the elbow-method (WCSS) curve.
    plt.subplot(1, 2, 2)
    plt.plot(k_values, wcss_list, marker='o')
    plt.title('Elbow Method (WCSS)')
    plt.xlabel('Number of Clusters (K)')
    plt.ylabel('Within-Cluster Sum of Squares (WCSS)')
    plt.grid(True)
    plt.xticks(k_values)
    plt.savefig(os.path.join(output_dir, 'elbow_method_wcss.png'))
    print(f"Elbow-method (WCSS) curve saved to {output_dir}/elbow_method_wcss.png")

    plt.tight_layout()
    plt.show()

def save_data_to_file(k_values, silhouette_scores, wcss_list, filename='clustering_results.txt', output_dir='results'):
    """
    Save the computed metrics to a text file.
    """
    if k_values is None:
        return
    
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    filepath = os.path.join(output_dir, filename)
    data_array = np.vstack((k_values, silhouette_scores, wcss_list)).T
    np.savetxt(filepath, data_array, fmt='%.4f', delimiter=',', 
               header='K_values,Silhouette_Score,WCSS', comments='')
    print(f"Clustering result data saved to {filepath}")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Evaluate candidate cluster counts for DCI vectors.")
    parser.add_argument("consistency_filename", help="Path to the similarity-value file.")
    parser.add_argument("stability_filename", help="Path to the stability-value file.")
    args = parser.parse_args()

    # Read, merge, and normalize the DCI data.
    normalized_dci_vectors = read_and_normalize_dci_data(args.consistency_filename, args.stability_filename)

    if normalized_dci_vectors is not None:
        # Compute metrics across the K range.
        k_values, silhouette_scores, wcss_list = find_optimal_k(normalized_dci_vectors)
        
        # Plot and save the figures.
        plot_and_save_results(k_values, silhouette_scores, wcss_list)
        
        # Save the numeric results.
        save_data_to_file(k_values, silhouette_scores, wcss_list)
