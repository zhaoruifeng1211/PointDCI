"""Compare two point clouds through multi-view silhouette differences."""

import argparse
import open3d as o3d
import numpy as np
from skimage.morphology import convex_hull_image
from shapely.geometry import Point, MultiPoint, Polygon
from shapely.ops import unary_union, polygonize
from scipy.spatial import Delaunay
from PIL import Image
import os
import cv2 # Strategy 1A: import OpenCV for efficient polygon filling.
import concurrent.futures # Reserved for future parallelization; the core logic is unchanged.

def alpha_shape(points, alpha):
    """
    Compute the alpha shape (concave hull) of a set of 2D points.
    """
    if len(points) < 4:
        # If fewer than four points are available, return the convex hull directly.
        return MultiPoint(list(points)).convex_hull

    # Delaunay triangulation.
    tri = Delaunay(points)
    edges = set()
    edge_points = []

    # Keep edges from triangles whose circumradius passes the alpha test.
    # Check the circumradius.
    for ia, ib, ic in tri.simplices:
        pa, pb, pc = points[ia], points[ib], points[ic]
        a = np.linalg.norm(pb - pc)
        b = np.linalg.norm(pa - pc)
        c = np.linalg.norm(pa - pb)
        s = (a + b + c) / 2.0
        
        # Compute the triangle area.
        area = max(s * (s - a) * (s - b) * (s - c), 0.0) ** 0.5
        
        # Compute the circumradius with R = abc / (4 * Area).
        # Add 1e-12 to avoid division by zero.
        circum_r = a * b * c / (4.0 * area + 1e-12) 
        
        # Keep the triangle edges when R < 1 / alpha.
        if circum_r < 1.0 / alpha:
            edges.update([(ia, ib), (ib, ic), (ic, ia)])

    for i, j in edges:
        # Note: Delaunay indices are relative to the `points` array.
        edge_points.append(points[[i, j]])

    if len(edge_points) == 0:
        return MultiPoint(list(points)).convex_hull

    # Merge all accepted edges into polygons.
    # Note: polygonize may return Polygon, MultiPolygon, LineString, etc.
    triangles = list(polygonize(edge_points))
    return unary_union(triangles)


def pointcloud_to_silhouette(pcd, view, img_size=256, method="alpha", alpha=0.02, max_alpha_points=5000):
    """
    Project point cloud to silhouette mask from a given view direction.
    method: "none" | "convex" | "alpha"
    max_alpha_points: maximum sample count used for alpha shape (Strategy 2)
    """
    points = np.asarray(pcd.points)
    if len(points) == 0:
        return np.zeros((img_size, img_size), dtype=np.uint8)

    # Normalize the cloud so different inputs share a comparable image footprint.
    # 1. Normalize.
    points = points - points.mean(axis=0)
    points /= np.max(np.linalg.norm(points, axis=1))

    # 2. View transformation (projection).
    view = view / np.linalg.norm(view)
    up = np.array([0, 1, 0]) if abs(view[1]) < 0.9 else np.array([1, 0, 0])
    right = np.cross(up, view)
    up = np.cross(view, right)
    R = np.stack([right, up, view], axis=1)
    proj = points @ R[:, :2]

    # 3. Normalize projection to image grid
    proj_min = proj.min(axis=0)
    proj_max = proj.max(axis=0)
    # Avoid division by zero.
    scale = (proj_max - proj_min + 1e-8)
    proj = (proj - proj_min) / scale
    proj *= (img_size - 1)
    
    # Convert to integer pixel coordinates.
    proj = proj.astype(np.int32)
    proj = np.clip(proj, 0, img_size - 1)

    # 4. Initialize and fill the mask.
    mask = np.zeros((img_size, img_size), dtype=np.uint8)
    # Scatter projection gives the base silhouette.
    mask[proj[:, 1], proj[:, 0]] = 1

    if method == "convex":
        # Convex hull.
        mask = convex_hull_image(mask)

    elif method == "alpha":
        points_2d = np.unique(proj, axis=0)
        
        # *** Strategy 2: downsample projected 2D points ***
        if len(points_2d) > max_alpha_points:
            indices = np.random.choice(len(points_2d), max_alpha_points, replace=False)
            points_2d = points_2d[indices]

        # Compute the alpha shape.
        polygon = alpha_shape(points_2d, alpha)
        
        # Rasterize the polygon efficiently instead of testing pixels one by one.
        # *** Strategy 1A: fill polygons efficiently with OpenCV ***
        if polygon is not None and (polygon.geom_type == 'Polygon' or polygon.geom_type == 'MultiPolygon'):
            
            polygons_to_fill = [polygon] if polygon.geom_type == 'Polygon' else polygon.geoms
            
            # Extract exterior boundary coordinates.
            exteriors = []
            for p in polygons_to_fill:
                # Ensure coordinates are integers (`pointcloud_to_silhouette` already casts them).
                # `exterior.coords` returns (x, y) coordinates.
                coords = np.array(p.exterior.coords, dtype=np.int32)
                # OpenCV expects shape (N, 1, 2).
                exteriors.append(coords[:, :2].reshape((-1, 1, 2))) 
            
            # Use `cv2.fillPoly`; this is much faster than per-pixel filling.
            if exteriors:
                # In OpenCV, y (rows) is the first dimension and x (columns) is the second.
                # Shapely and OpenCV coordinates are typically compatible for this rasterization step.
                cv2.fillPoly(mask, exteriors, 1)

    return mask


def silhouette_difference(pcd1, pcd2, num_views=50, img_size=256, method="alpha", alpha=0.02, max_alpha_points=5000):
    """
    Compute silhouette difference between two point clouds across multiple views.
    """
    np.random.seed(0)
    views = np.random.normal(size=(num_views, 3))
    diffs = []

    for i, v in enumerate(views):
        # Pass the `max_alpha_points` parameter through.
        mask1 = pointcloud_to_silhouette(pcd1, v, img_size, method, alpha, max_alpha_points)
        mask2 = pointcloud_to_silhouette(pcd2, v, img_size, method, alpha, max_alpha_points)
        
        # Compute the Jaccard-style difference (1 - IoU).
        union = np.sum(np.logical_or(mask1, mask2))
        if union == 0:
             # If both masks are empty, the difference is 0.
             diff = 0.0
        else:
             intersection_complement = np.sum(np.logical_xor(mask1, mask2))
             diff = intersection_complement / union 
             
        diffs.append(diff)
        print(f"View {i+1}: silhouette difference = {diff:.4f}")

    mean_diff = np.mean(diffs)
    print(f"\nAverage silhouette difference over {num_views} views: {mean_diff:.4f}")
    return diffs, mean_diff


def remove_black_points_from_pcd(pcd):
    """Remove points whose color is effectively black."""
    if not pcd.has_colors():
        return pcd
    colors = np.asarray(pcd.colors)
    non_black_mask = np.any(colors > 0.05, axis=1)  # Allow small floating-point error.
    pcd.points = o3d.utility.Vector3dVector(np.asarray(pcd.points)[non_black_mask])
    pcd.colors = o3d.utility.Vector3dVector(colors[non_black_mask])
    return pcd


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Compare two point clouds with multi-view silhouette differences.")
    parser.add_argument("pcd1_path", help="Path to the first point cloud.")
    parser.add_argument("pcd2_path", help="Path to the second point cloud.")
    parser.add_argument("--num-views", type=int, default=50, help="Number of camera views.")
    parser.add_argument("--img-size", type=int, default=256, help="Raster size for each silhouette.")
    parser.add_argument("--method", default="alpha", help="Silhouette method.")
    parser.add_argument("--alpha", type=float, default=0.02, help="Alpha-shape parameter.")
    parser.add_argument("--max-alpha-points", type=int, default=5000, help="Downsample threshold for alpha-shape processing.")
    args = parser.parse_args()

    print(f"Loading {args.pcd1_path}...")
    pcd1 = o3d.io.read_point_cloud(args.pcd1_path)
    pcd1 = remove_black_points_from_pcd(pcd1)
    
    print(f"Loading {args.pcd2_path}...")
    pcd2 = o3d.io.read_point_cloud(args.pcd2_path)

    print("Starting silhouette difference calculation with Alpha Shape optimization (OpenCV + Downsampling)...")
    per_view_diff, mean_diff = silhouette_difference(
        pcd1, pcd2,
        num_views=args.num_views,
        img_size=args.img_size,
        method=args.method,
        alpha=args.alpha,
        max_alpha_points=args.max_alpha_points
    )

    # Output file path.
    output_dir = os.path.dirname(args.pcd1_path)
    base_name = os.path.splitext(os.path.basename(args.pcd1_path))[0]
    out_path = os.path.join(output_dir, f"{base_name}_silhouette_difference_optimized.txt")

    with open(out_path, "w") as f:
        for i, d in enumerate(per_view_diff):
            f.write(f"View {i+1}: {d:.4f}\n")
        f.write(f"\nAverage: {mean_diff:.4f}\n")

    print(f"\nSaved results to: {out_path}")
