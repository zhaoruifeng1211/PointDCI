"""Step through sampled points interactively and center the camera on each one."""

import argparse
import open3d as o3d
import numpy as np
import pandas as pd
import os
import sys
import copy
import time

class InteractiveViewer:
    """
    Interactive viewer: load point clouds and an index CSV, then highlight points
    and switch the camera view sequentially.
    """
    def __init__(self, original_pcd_path, sampled_pcd_path, csv_path, marker_scale=0.01):
        self.original_pcd_path = original_pcd_path
        self.sampled_pcd_path = sampled_pcd_path
        self.csv_path = csv_path
        self.marker_scale = marker_scale
        
        self.pcd_original = None
        self.pcd_sampled = None
        self.df_indices = None
        
        self.current_index_df = 0 # Current row index in the DataFrame.
        self.n_samples = 0
        
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.current_marker = None
        self.pcd_original_vis = None # Original point-cloud object used for visualization.

    def load_data(self):
        """Load the point clouds and the index CSV file."""
        print(f"1. Loading original point cloud: {self.original_pcd_path}")
        try:
            # Keep a gray copy of the original cloud as the viewing background.
            self.pcd_original = o3d.io.read_point_cloud(self.original_pcd_path)
            self.pcd_original_vis = copy.deepcopy(self.pcd_original)
            self.pcd_original_vis.paint_uniform_color([0.7, 0.7, 0.7]) # Gray background.
        except Exception as e:
            print(f"Failed to load the original point cloud: {e}")
            return False

        print(f"2. Loading sampled point cloud: {self.sampled_pcd_path}")
        try:
            self.pcd_sampled = o3d.io.read_point_cloud(self.sampled_pcd_path)
            self.pcd_sampled.paint_uniform_color([1, 0, 0]) # Red sampled points.
        except Exception as e:
            print(f"Failed to load the sampled point cloud: {e}")
            return False

        print(f"3. Loading index CSV: {self.csv_path}")
        try:
            self.df_indices = pd.read_csv(self.csv_path)
            self.n_samples = len(self.df_indices)
        except Exception as e:
            print(f"Failed to load the index CSV: {e}")
            return False
            
        print(f"Loaded {self.n_samples} sampled points for inspection.")
        return True

    def _get_current_point_data(self):
        """Get the coordinates and original index of the current point."""
        if self.current_index_df >= self.n_samples:
            return None
        
        row = self.df_indices.iloc[self.current_index_df]
        coords = np.array([row['x'], row['y'], row['z']])
        original_index = row['original_point_index']
        sampled_id = row['sampled_point_id']
        
        return coords, original_index, sampled_id

    def _create_marker(self, point_coords):
        """Create the highlight sphere marker."""
        # Estimate a suitable marker radius from the point-cloud bounds.
        bounds = self.pcd_original.get_max_bound() - self.pcd_original.get_min_bound()
        radius = self.marker_scale * np.linalg.norm(bounds) / np.sqrt(3) 
        
        marker = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
        marker.translate(point_coords)
        marker.paint_uniform_color([0, 0, 1]) # Blue highlight.
        return marker
        
    def _update_view_and_marker(self):
        """Update the highlight marker and camera view."""
        if self.current_index_df >= self.n_samples:
            print("All sampled points have been inspected.")
            return False
            
        coords, original_index, sampled_id = self._get_current_point_data()
        
        print(f"\n--- Showing point {self.current_index_df + 1}/{self.n_samples} (original index: {original_index}) ---")
        print(f"   Coordinates: {coords}")

        # 1. Remove the previous marker, if it exists.
        if self.current_marker is not None:
            self.vis.remove_geometry(self.current_marker, reset_bounding_box=False)

        # 2. Create and add the new marker.
        self.current_marker = self._create_marker(coords)
        self.vis.add_geometry(self.current_marker, reset_bounding_box=False)
        
        # Move the camera target to the active sampled point.
        # 3. Move the camera view.
        ctr = self.vis.get_view_control()
        ctr.set_lookat(coords)
        ctr.set_zoom(0.5) # Apply a moderate zoom.
        
        self.vis.update_renderer()
        return True

    def run(self):
        """Main entry point: initialize and enter the visualization loop."""
        if not self.load_data():
            return

        # --- 4. Initialize the visualizer and its geometries ---
        self.vis.create_window(window_name="Interactive Point-Cloud Viewer - Press [Tab] to switch", width=1024, height=768)
        
        # Add all geometries.
        self.vis.add_geometry(self.pcd_original_vis) # Original point cloud (gray background).
        self.vis.add_geometry(self.pcd_sampled)      # Sampled points (red).

        # Register keyboard shortcuts for stepping through the sample list.
        # Define key callbacks.
        # KeyCode.TAB = 258 in most systems and Open3D versions.
        def key_callback_tab(vis):
            # Move to the next point.
            self.current_index_df += 1
            if not self._update_view_and_marker():
                vis.close()
            return False

        # KeyCode.ESC = 256 (exit).
        def key_callback_esc(vis):
            print("\nExit requested by the user.")
            vis.close()
            return False

        # Register callbacks.
        print("4. Visualization window started.")
        print("   - Press [Tab] or [T] to move to the next sampled point.")
        print("   - Press [ESC] to close the window and exit.")
        
        self.vis.register_key_callback(258, key_callback_tab) # Register the TAB key.
        self.vis.register_key_callback(84, key_callback_tab)  # Register T as a fallback.

        # Update the first view.
        if not self._update_view_and_marker():
            print("There are no points to display.")
            self.vis.destroy_window()
            return

        # Run in blocking mode until the window closes or all points are visited.
        self.vis.run() 
        
        # Clean up.
        self.vis.destroy_window()
        print("\nThe viewer has been closed. Please refer to the CSV file for follow-up evaluation.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Interactively inspect sampled points from prepared evaluation data.")
    parser.add_argument("original_ply", help="Path to the original point cloud.")
    parser.add_argument("sampled_ply", help="Path to the sampled point cloud.")
    parser.add_argument("csv_path", help="Path to the CSV index table from data_sampler.py.")
    args = parser.parse_args()

    # 1. Make sure the data files exist.
    if not os.path.exists(args.sampled_ply) or not os.path.exists(args.csv_path):
        print("Error: sampled data files do not exist. Run data_sampler.py first to prepare them.")
        sys.exit(1)
        
    # 2. Run the viewer.
    try:
        viewer = InteractiveViewer(
            original_pcd_path=args.original_ply,
            sampled_pcd_path=args.sampled_ply,
            csv_path=args.csv_path
        )
        viewer.run()
    except Exception as e:
        print(f"\nAn error occurred while running the viewer: {e}")
        sys.exit(1)
