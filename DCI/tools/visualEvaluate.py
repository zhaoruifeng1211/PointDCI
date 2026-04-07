"""Interactively inspect sampled points in a cloud and record manual ratings."""

import argparse
import open3d as o3d
import numpy as np
import json
import copy
import sys
import os

class PointCloudEvaluator:
    """
    Interactive point-cloud evaluator: read a PLY file, sample points, visualize
    them one by one, collect user ratings, and save the results.
    """
    def __init__(self, pcd_path, n_samples=100, marker_scale=0.01, output_file="evaluation_results.json"):
        self.pcd_path = pcd_path
        self.n_samples = n_samples
        self.marker_scale = marker_scale  # Scale factor for the highlight sphere radius.
        self.output_file = output_file
        
        self.point_cloud = None
        self.sampled_indices = None
        self.evaluation_data = {}
        
        self.current_index = 0
        self.is_running = True # Flag indicating whether the program should keep running.
        
        # Open3D geometries.
        self.background_pcd = None
        self.sampled_pcd = None
        self.current_marker = None
        
        # Persistent Open3D visualizer instance.
        self.vis = o3d.visualization.VisualizerWithKeyCallback()

    def load_and_sample(self):
        """Read the point cloud and sample points; random sampling is used here as an FPS placeholder."""
        print(f"1. Reading point-cloud file: {self.pcd_path}")
        if not os.path.exists(self.pcd_path):
            print(f"Error: file not found: {self.pcd_path}")
            return False
            
        try:
            self.point_cloud = o3d.io.read_point_cloud(self.pcd_path)
        except Exception as e:
            print(f"Failed to read point cloud: {e}")
            return False

        points_count = len(self.point_cloud.points)
        if not self.point_cloud.has_points() or points_count < self.n_samples:
            print(f"Point-cloud size ({points_count}) is insufficient or loading failed.")
            return False

        print(f"Original point count: {points_count}")

        # This script currently uses random sampling as a lightweight placeholder.
        # --- 2. Sampling (random sampling used here as an FPS placeholder) ---
        points = np.asarray(self.point_cloud.points)
        all_indices = np.arange(len(points))
        # Make sure the sample count does not exceed the total point count.
        num_samples_actual = min(self.n_samples, points_count)
        self.sampled_indices = np.random.choice(all_indices, num_samples_actual, replace=False)
        self.n_samples = num_samples_actual # Update the actual sample count.
        print(f"2. Completed sampling of {self.n_samples} points (random sampling used instead of FPS).")
        
        # Prepare the sampled point cloud for separate coloring and display.
        self.sampled_pcd = self.point_cloud.select_by_index(self.sampled_indices)
        self.sampled_pcd.paint_uniform_color([1, 0, 0]) # Red.

        return True

    def _get_current_sampled_point(self):
        """Get the coordinates of the current sampled point."""
        if self.sampled_indices is None or self.current_index >= len(self.sampled_indices):
            return None
        
        point_idx = self.sampled_indices[self.current_index]
        return np.asarray(self.point_cloud.points)[point_idx]

    def _update_view_to_point(self, point_coords):
        """Move the camera focus to the given point."""
        ctr = self.vis.get_view_control()
        
        # Set the camera target to the current point.
        ctr.set_lookat(point_coords)
        
        # Apply a moderate zoom; this may need tuning for your cloud scale.
        ctr.set_zoom(0.5) 
        
        # Force the renderer to update.
        self.vis.update_renderer()

    def _create_marker(self, point_coords):
        """Create the highlight sphere marker."""
        # Estimate a suitable sphere radius from the point-cloud bounds.
        bounds = self.point_cloud.get_max_bound() - self.point_cloud.get_min_bound()
        radius = self.marker_scale * np.linalg.norm(bounds) / np.sqrt(3) # Normalize by diagonal length.
        
        marker = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
        marker.translate(point_coords)
        marker.paint_uniform_color([0, 0, 1]) # Blue highlight.
        return marker

    def get_user_ratings(self, point_coords):
        """Receive two rating values from the command line."""
        print("\n" + "="*50)
        print(f"3. Current sampled point ({self.current_index + 1}/{self.n_samples}): coordinates {point_coords}")
        print("Please enter the rating values in the terminal.")
        
        rating1 = None
        rating2 = None
        
        while rating1 is None:
            try:
                r1_str = input("Enter rating 1 (number): ")
                rating1 = float(r1_str)
            except ValueError:
                print("Invalid input. Please enter a number.")
        
        while rating2 is None:
            try:
                r2_str = input("Enter rating 2 (number): ")
                rating2 = float(r2_str)
            except ValueError:
                print("Invalid input. Please enter a number.")

        print("="*50 + "\n")
        return rating1, rating2

    def save_results(self):
        """Save rating results to a local file."""
        if not self.evaluation_data:
            print("No evaluation data available to save.")
            return
            
        with open(self.output_file, 'w') as f:
            json.dump(self.evaluation_data, f, indent=4)
        print(f"5. All ratings have been saved to: {self.output_file}")

    def run(self):
        """Main loop: alternate between visualization and console input."""
        if not self.load_and_sample():
            self.vis.destroy_window()
            return

        # Keep the viewer alive across multiple rating rounds.
        # --- 4. Initialize the persistent visualizer and geometries ---
        self.vis.create_window(window_name="Point Cloud Evaluator - Press [T] to move on", width=1024, height=768)
        
        # Add the background point cloud in gray.
        self.background_pcd = copy.deepcopy(self.point_cloud)
        self.background_pcd.paint_uniform_color([0.7, 0.7, 0.7])
        self.vis.add_geometry(self.background_pcd)
        # Add sampled points in red.
        self.vis.add_geometry(self.sampled_pcd)

        # Register keyboard callbacks.
        
        # T key (84): close the window so the main loop can continue.
        def key_t_callback(vis):
            vis.close()
            return False
        
        # ESC key (256): set the exit flag and close the window.
        def key_esc_callback(vis):
            print("\nExit requested with the ESC key.")
            self.is_running = False
            vis.close()
            return False

        self.vis.register_key_callback(84, key_t_callback) # T key.
        self.vis.register_key_callback(256, key_esc_callback) # ESC key.
        
        print("4. Visualization window started.")
        print("   - Inspect the highlighted point and adjust the view.")
        print("   - When finished, press 'T' to close the window and enter ratings.")
        print("   - Press 'ESC' to exit early and save the current progress.")

        self.current_index = 0
        while self.is_running and self.current_index < self.n_samples:
            
            point_coords = self._get_current_sampled_point()
            current_original_index = self.sampled_indices[self.current_index]
            
            print(f"\n--- Processing point {self.current_index + 1}/{self.n_samples} (original index: {current_original_index}) ---")

            # --- A. Update highlight geometry and camera view ---
            
            # 1. Remove the previous marker, if any.
            if self.current_marker is not None:
                self.vis.remove_geometry(self.current_marker, reset_bounding_box=False)

            # 2. Create and add the new marker.
            self.current_marker = self._create_marker(point_coords)
            self.vis.add_geometry(self.current_marker, reset_bounding_box=False)
            
            # 3. Update the camera view.
            self._update_view_to_point(point_coords)

            # --- B. Block and wait for user interaction ---
            
            # Each call to `run()` blocks until the window closes or a key callback triggers `vis.close()`.
            self.vis.run() 
            
            # Stop here if the user pressed ESC or all points have been processed.
            if not self.is_running:
                break
                
            # Important: detect whether `vis.run()` ended because of 'T' or because the user clicked 'X'.
            # If the window was closed with 'X', recreate it.
            if not self.vis.poll_events(): 
                print("The window was closed with X; attempting to recreate it for input...")
                
                # Recreate the window.
                self.vis.create_window(window_name="Point Cloud Evaluator - Press [T] to move on", width=1024, height=768)
                
                # Re-add geometries and callbacks.
                self.vis.add_geometry(self.background_pcd, reset_bounding_box=False)
                self.vis.add_geometry(self.sampled_pcd, reset_bounding_box=False)
                self.vis.add_geometry(self.current_marker, reset_bounding_box=False)
                self.vis.register_key_callback(84, key_t_callback)
                self.vis.register_key_callback(256, key_esc_callback)
                
                # Reset the view if needed; otherwise the last view is kept.
                self._update_view_to_point(point_coords)
                
            # --- C. Receive user input (only after `run()` returns) ---
            r1, r2 = self.get_user_ratings(point_coords)

            # --- D. Record the result ---
            # Store both coordinates and manual scores for later analysis.
            point_key = f"{current_original_index:07d}" # Use a zero-padded index as the key.
            self.evaluation_data[point_key] = {
                "original_index": int(current_original_index),
                "coordinates": point_coords.tolist(),
                "rating_1": r1,
                "rating_2": r2
            }
            
            self.current_index += 1 

        # Clean up resources after the loop.
        if self.vis.poll_events(): # Check whether the window still exists.
             self.vis.destroy_window()

        # 5. Save results.
        self.save_results()

# Example usage.
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Interactively evaluate sampled points in a point cloud.")
    parser.add_argument("ply_file", help="Path to the input point cloud.")
    parser.add_argument("--n-samples", type=int, default=100, help="Number of sampled points.")
    parser.add_argument("--output-file", default="evaluation_results.json", help="Output JSON file for manual ratings.")
    args = parser.parse_args()

    try:
        evaluator = PointCloudEvaluator(
            pcd_path=args.ply_file, 
            n_samples=args.n_samples, 
            output_file=args.output_file
        ) 
        evaluator.run()
    except Exception as e:
        print(f"\nA fatal error occurred while running the program: {e}")
        # Force-destroy the visualizer to avoid leftover resources.
        try:
            evaluator.vis.destroy_window()
        except:
            pass
        sys.exit(1)
        
    print("\nThe program exited normally.")
