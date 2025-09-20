import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import copy
from sklearn.cluster import KMeans
import os
import tempfile
import CSF
import shapely.geometry as geometry
from scipy.spatial import ConvexHull 

def get_rotation_matrix_from_vectors(vec1, vec2):
    """
    Calculate rotation matrix to rotate vec1 to vec2 direction.
    
    Args:
        vec1: Source vector
        vec2: Target vector
        
    Returns:
        3x3 rotation matrix
    """
    a = vec1 / np.linalg.norm(vec1)
    b = vec2 / np.linalg.norm(vec2)
    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)
    vx = np.array([[0, -v[2], v[1]],
                   [v[2], 0, -v[0]],
                   [-v[1], v[0], 0]])
    R = np.eye(3) + vx + np.dot(vx, vx) * ((1 - c) / (s**2 + 1e-10))
    return R

def create_lineset_from_mesh(mesh):
    """
    Generate convex hull boundary lines from a triangle mesh for visualization.
    
    Args:
        mesh: Open3D triangle mesh
        
    Returns:
        Open3D LineSet object with red color
    """
    triangles = np.asarray(mesh.triangles)
    vertices = np.asarray(mesh.vertices)
    edges = set()
    for tri in triangles:
        edges.add(tuple(sorted((tri[0], tri[1]))))
        edges.add(tuple(sorted((tri[1], tri[2]))))
        edges.add(tuple(sorted((tri[2], tri[0]))))
    edges = list(edges)
    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(vertices),
        lines=o3d.utility.Vector2iVector(edges)
    )
    line_set.paint_uniform_color([1, 0, 0])  # Red for convex hull
    return line_set

def create_oobb_lineset(oobb):
    """
    Create a line set for oriented bounding box visualization.
    
    Args:
        oobb: Open3D OrientedBoundingBox object
        
    Returns:
        Open3D LineSet object with blue color
    """
    line_set = o3d.geometry.LineSet.create_from_oriented_bounding_box(oobb)
    line_set.paint_uniform_color([0, 0, 1])  # Blue for OOBB
    return line_set

def remove_noise(pcd, nb_neighbors=20, std_ratio=2.0):
    """
    Remove noise using statistical outlier removal.
    
    Args:
        pcd: Input point cloud
        nb_neighbors: Number of neighbors to consider
        std_ratio: Standard deviation ratio threshold
        
    Returns:
        Filtered point cloud
    """
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    return cl

def remove_cluster_noise(pcd, eps=0.05, min_cluster_size=100):
    """
    Remove small noise clusters using DBSCAN clustering.
    Only clusters with more than min_cluster_size points are kept.
    
    Args:
        pcd: Input point cloud
        eps: DBSCAN epsilon parameter
        min_cluster_size: Minimum cluster size to keep
        
    Returns:
        Filtered point cloud
    """
    labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=10, print_progress=True))
    unique_labels, counts = np.unique(labels, return_counts=True)
    good_labels = [label for label, count in zip(unique_labels, counts)
                   if label != -1 and count >= min_cluster_size]
    indices = [i for i, lab in enumerate(labels) if lab in good_labels]
    filtered_pcd = pcd.select_by_index(indices)
    return filtered_pcd

def remove_landmark_sign(pcd, eps=0.1, min_points=20, area_threshold=0.2):
    """
    Remove small clusters like landmark signs using DBSCAN clustering and convex hull area filtering.
    Clusters with convex hull area below area_threshold are considered as landmark signs and removed.
    
    Args:
        pcd: Input point cloud
        eps: DBSCAN epsilon parameter
        min_points: Minimum points for DBSCAN
        area_threshold: Minimum convex hull area to keep
        
    Returns:
        Filtered point cloud
    """
    labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=True))
    unique_labels = np.unique(labels)
    good_indices = []
    for label in unique_labels:
        if label == -1:
            continue
        idx = np.where(labels == label)[0]
        cluster = pcd.select_by_index(idx)
        try:
            hull, _ = cluster.compute_convex_hull()
            area = hull.get_surface_area()
        except Exception as e:
            print(f"Error computing convex hull for cluster {label}: {e}")
            area = 0
        if area >= area_threshold:
            good_indices.extend(idx.tolist())
    filtered_pcd = pcd.select_by_index(good_indices)
    return filtered_pcd

def csf_soil_segmentation(pcd, cloth_resolution=0.5, bSloopSmooth=False):
    """
    Segment ground points using Cloth Simulation Filter (CSF) method.
    
    Args:
        pcd: Input point cloud
        cloth_resolution: Resolution of the cloth grid
        bSloopSmooth: Whether to apply slope smoothing
        
    Returns:
        Tuple of (soil_cloud, canopy_cloud)
    """
    tmp_file = tempfile.NamedTemporaryFile(delete=False, suffix=".txt")
    np.savetxt(tmp_file.name, np.asarray(pcd.points), fmt="%.6f")
    tmp_file.close()
    
    csf = CSF.CSF()
    csf.readPointsFromFile(tmp_file.name)
    
    csf.params.bSloopSmooth = bSloopSmooth
    csf.params.cloth_resolution = cloth_resolution
    csf.params.rigidness = 3
    csf.params.time_step = 0.65
    csf.params.class_threshold = 0.12
    csf.params.interations = 500

    ground = CSF.VecInt()
    non_ground = CSF.VecInt()
    csf.do_filtering(ground, non_ground)
    
    soil_cloud = pcd.select_by_index([int(i) for i in ground])
    canopy_cloud = pcd.select_by_index([int(i) for i in non_ground])
    
    os.remove(tmp_file.name)
    
    return soil_cloud, canopy_cloud

def segment_major_rows(plot_cloud, n_rows=3, planting_direction=None):
    """
    Segment canopy point cloud into n_rows rows.
    
    Args:
        plot_cloud: Input point cloud
        n_rows: Number of rows to segment
        planting_direction: Optional planting direction vector [x, y]
        
    Returns:
        Tuple of (major_rows, ordered_labels)
        - major_rows: List of (row_id, row_point_cloud) tuples
        - ordered_labels: Array of row labels for each point
    """
    points = np.asarray(plot_cloud.points)
    if points.shape[0] == 0:
        return [], None
    xy = points[:, :2]
    if planting_direction is None:
        # Calculate planting direction using PCA
        mean_xy = np.mean(xy, axis=0)
        centered = xy - mean_xy
        U, S, Vt = np.linalg.svd(centered, full_matrices=False)
        row_direction = Vt[0]
    else:
        row_direction = np.array(planting_direction, dtype=float)
        row_direction = row_direction / np.linalg.norm(row_direction)
    
    # Calculate separation axis perpendicular to row direction
    separation_axis = np.array([-row_direction[1], row_direction[0]])
    separation_axis = separation_axis / np.linalg.norm(separation_axis)
    print("Separation axis (perpendicular to row direction):", separation_axis)
    
    # Project points onto separation axis
    projections = xy.dot(separation_axis)
    
    # Apply K-means clustering
    kmeans = KMeans(n_clusters=n_rows, random_state=0).fit(projections.reshape(-1, 1))
    labels = kmeans.labels_
    
    # Order clusters by their mean projection
    cluster_means = [projections[labels == i].mean() for i in range(n_rows)]
    order = np.argsort(cluster_means)
    ordered_labels = np.zeros_like(labels)
    for new_label, orig_label in enumerate(order):
        ordered_labels[labels == orig_label] = new_label
    
    # Create point clouds for each row
    major_rows = []
    for i in range(n_rows):
        indices = np.where(ordered_labels == i)[0]
        row_pc = plot_cloud.select_by_index(indices)
        major_rows.append((i, row_pc))
    
    return major_rows, ordered_labels

def manual_remove_points(pcd):
    """
    Allow user to manually select and remove points.
    When 3 or more points are selected, removes all points within the convex hull
    of selected points projected onto XY plane.
    
    Args:
        pcd: Input point cloud
        
    Returns:
        Filtered point cloud
    """
    print("Please select points to remove in the popup window by holding Shift and clicking.")
    print("Select multiple points to remove entire clusters. Press 'q' to exit.")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name="Manual Point Removal: Select points to remove, press q to exit")
    vis.add_geometry(pcd)
    vis.run()  # User interaction happens here
    picked_ids = vis.get_picked_points()
    vis.destroy_window()
    print("Selected point indices:", picked_ids)
    
    if picked_ids is not None and len(picked_ids) > 0:
        pts = np.asarray(pcd.points)
        if len(picked_ids) >= 3:
            # Use selected points to calculate 2D convex hull, then remove all points inside
            selected_pts = pts[picked_ids, :2]  # Get XY coordinates
            try:
                hull = ConvexHull(selected_pts)
                hull_pts = selected_pts[hull.vertices]
                poly = geometry.Polygon(hull_pts)
                remove_indices = []
                for i, p in enumerate(pts[:, :2]):  # Check all points' XY coordinates
                    if poly.contains(geometry.Point(p)):
                        remove_indices.append(i)
                print(f"Cluster removal: Will remove {len(remove_indices)} points.")
                pcd_out = pcd.select_by_index(remove_indices, invert=True)
                return pcd_out
            except Exception as e:
                print("Failed to compute convex hull, falling back to point-by-point removal:", e)
                pcd_out = pcd.select_by_index(picked_ids, invert=True)
                print(f"Point-by-point removal: Removed {len(picked_ids)} points.")
                return pcd_out
        else:
            # If few points selected, just remove them individually
            pcd_out = pcd.select_by_index(picked_ids, invert=True)
            print(f"Point-by-point removal: Removed {len(picked_ids)} points.")
            return pcd_out
    else:
        print("No points selected, returning original point cloud.")
        return pcd

def compute_mesh_volume(mesh):
    """
    Calculate volume of a triangle mesh using divergence theorem.
    
    Args:
        mesh: Open3D triangle mesh
        
    Returns:
        Volume of the mesh
    """
    vertices = np.asarray(mesh.vertices)
    triangles = np.asarray(mesh.triangles)
    volume = 0.0
    for tri in triangles:
        p1, p2, p3 = vertices[tri]
        volume += np.dot(p1, np.cross(p2, p3))
    return abs(volume) / 6.0

def fill_lower_surface(row_cloud, num_points=1000, fill_z=None):
    """
    Fill the lower surface of a point cloud using its XY convex hull.
    Creates a grid of points at the specified z-height to close the bottom.
    
    Args:
        row_cloud: Input point cloud
        num_points: Number of points to generate for filling
        fill_z: Z-coordinate for filling points (uses min z if None)
        
    Returns:
        Point cloud with filled bottom surface
    """
    points = np.asarray(row_cloud.points)
    if fill_z is None:
        fill_z = np.min(points[:, 2])
    xy = points[:, :2]
    
    # Calculate 2D convex hull using Shapely
    pcd_xy = o3d.geometry.PointCloud()
    pcd_xy.points = o3d.utility.Vector3dVector(np.hstack([xy, np.zeros((xy.shape[0], 1))]))
    hull, _ = pcd_xy.compute_convex_hull()
    hull_pts = np.asarray(hull.vertices)[:, :2]
    poly = geometry.Polygon(hull_pts)
    
    # Calculate polygon bounds and area
    minx, miny, maxx, maxy = poly.bounds
    poly_area = poly.area
    
    # Use regular grid sampling, calculate step size
    step = np.sqrt(poly_area / float(num_points))
    grid_x = np.arange(minx, maxx, step)
    grid_y = np.arange(miny, maxy, step)
    grid_points = np.array([[x, y] for x in grid_x for y in grid_y])
    
    # Filter points inside the polygon
    inside_points = np.array([pt for pt in grid_points if poly.contains(geometry.Point(pt))])
    
    # If no valid points found, add at least one fill point
    if inside_points.shape[0] == 0:
        inside_points = np.array([[minx, miny]])
    
    # Create 3D points at fill_z height
    fill_points = np.hstack([inside_points, np.full((inside_points.shape[0], 1), fill_z)])
    
    # Combine original and fill points
    new_points = np.vstack([points, fill_points])
    new_cloud = o3d.geometry.PointCloud()
    new_cloud.points = o3d.utility.Vector3dVector(new_points)
    return new_cloud

def process_point_cloud(file_path, planting_direction=None):
    """
    Complete processing pipeline for UAV-CCO reconstructed plant canopy point clouds:
    
    1. Load and visualize point cloud
    2. Segment soil using CSF algorithm
    3. Calculate ground plane, rotate and translate to correct the point cloud
    4. Remove noise from canopy candidate points using statistical and DBSCAN methods
    5. Segment canopy into three major rows and calculate metrics:
       - Volume ratio: convex hull volume / minimum cube volume
       - Canopy area ratio: (row projected area × 3) / original canopy projected area
       - Plant height: max height, min height, mean height, and height range
    
    Args:
        file_path: Path to input point cloud file (.ply)
        planting_direction: Optional planting direction vector [x, y]
        
    Returns:
        Dictionary containing processed point clouds and analysis results
    """
    # 1. Load and display original point cloud
    pcd = o3d.io.read_point_cloud(file_path)
    print("Loaded point cloud with", len(pcd.points), "points")
    o3d.visualization.draw_geometries([pcd], window_name="Original Point Cloud")
    
    # 2. Segment soil using CSF
    soil_cloud, canopy_cloud = csf_soil_segmentation(pcd, cloth_resolution=0.1, bSloopSmooth=False)
    print("Identified soil points:", len(soil_cloud.points))
    print("Initial canopy candidate points:", len(canopy_cloud.points))
    o3d.visualization.draw_geometries([soil_cloud], window_name="Soil Point Cloud")
    o3d.visualization.draw_geometries([canopy_cloud], window_name="Initial Canopy Candidate Point Cloud")
    
    # 3. Ground plane calculation and point cloud correction
    plane_model, _ = soil_cloud.segment_plane(distance_threshold=0.02, ransac_n=3, num_iterations=1000)
    print("Ground plane model: ax+by+cz+d =", plane_model)
    a, b, c, d_param = plane_model
    ground_normal = np.array([a, b, c])
    target_normal = np.array([0, 0, 1])
    R_mat = get_rotation_matrix_from_vectors(ground_normal, target_normal)
    print("Rotation matrix:\n", R_mat)
    
    pcd_rotated = copy.deepcopy(pcd)
    pcd_rotated.rotate(R_mat, center=(0, 0, 0))
    soil_cloud_rotated = copy.deepcopy(soil_cloud)
    canopy_cloud_rotated = copy.deepcopy(canopy_cloud)
    soil_cloud_rotated.rotate(R_mat, center=(0, 0, 0))
    canopy_cloud_rotated.rotate(R_mat, center=(0, 0, 0))
    soil_points = np.asarray(soil_cloud_rotated.points)
    min_z = np.min(soil_points[:, 2])
    print("Minimum z value of rotated soil cloud:", min_z)
    translation = np.array([0, 0, -min_z])
    pcd_rotated.translate(translation)
    soil_cloud_rotated.translate(translation)
    canopy_cloud_rotated.translate(translation)
    o3d.visualization.draw_geometries([pcd_rotated], window_name="Corrected Point Cloud")
    
    # 4. Canopy noise removal, cluster filtering, landmark sign removal
    canopy_clean = remove_noise(canopy_cloud_rotated, nb_neighbors=20, std_ratio=2.0)
    print("Canopy points after noise removal:", len(canopy_clean.points))
    canopy_filtered = remove_cluster_noise(canopy_clean, eps=0.05, min_cluster_size=100)
    print("Canopy points after small cluster removal:", len(canopy_filtered.points))
    canopy_filtered = remove_landmark_sign(canopy_filtered, eps=0.1, min_points=20, area_threshold=0.2)
    print("Canopy points after landmark sign removal:", len(canopy_filtered.points))
    o3d.visualization.draw_geometries([canopy_filtered], window_name="Filtered Canopy Point Cloud")
    
    # 5. Manual point removal (optional)
    canopy_filtered = manual_remove_points(canopy_filtered)
    o3d.visualization.draw_geometries([canopy_filtered], window_name="Canopy After Manual Removal")
    
    # 6. Segment canopy into three major rows and calculate metrics
    canopy_final = canopy_filtered
    canopy_final_xy = np.asarray(canopy_final.points)[:, :2]
    
    # Create 2D projection for area calculation
    epsilon = 1e-2
    random_noise = np.random.uniform(-1e-4, 1e-4, size=(canopy_final_xy.shape[0], 1))
    canopy_points_3d = np.hstack([canopy_final_xy, np.full((canopy_final_xy.shape[0], 1), epsilon) + random_noise])
    pcd_canopy_xy = o3d.geometry.PointCloud()
    pcd_canopy_xy.points = o3d.utility.Vector3dVector(canopy_points_3d)
    
    # Calculate original projected area
    try:
        hull_canopy, _ = pcd_canopy_xy.compute_convex_hull()
        original_projected_area = hull_canopy.get_surface_area()
        print("Original canopy projected area on XY plane:", original_projected_area)
    except Exception as e:
        print("Error computing original projected area:", e)
        original_projected_area = 0
    
    # Segment into rows
    major_rows, row_labels = segment_major_rows(canopy_final, n_rows=3, planting_direction=planting_direction)
    cmap_row = plt.get_cmap("Set1")
    all_results = []
    colored_rows = []
    
    for row_id, row_pc in major_rows:
        color = cmap_row(row_id % 9)[:3]
        row_pc.paint_uniform_color(color)
        colored_rows.append(row_pc)
        pts = np.asarray(row_pc.points)
        if pts.size == 0:
            continue
        
        # Height statistics
        height_max = pts[:, 2].max()
        height_min = pts[:, 2].min()
        height_mean = pts[:, 2].mean()
        plant_height = height_max - height_min
        
        # Fill bottom surface for volume calculation
        try:
            filled_row = fill_lower_surface(row_pc)
            row_hull, _ = filled_row.compute_convex_hull()
            convex_hull_volume = compute_mesh_volume(row_hull)
        except Exception as e:
            print(f"Error computing convex hull volume for row {row_id}:", e)
            convex_hull_volume = None
        
        # Calculate minimum cube volume (using maximum OOBB dimension)
        oobb = row_pc.get_oriented_bounding_box()
        dims = oobb.extent
        side = np.max(dims)
        minimal_cube_volume = side ** 3
        
        # Calculate volume ratio
        if convex_hull_volume is not None and minimal_cube_volume > 0:
            volume_ratio = convex_hull_volume / minimal_cube_volume
        else:
            volume_ratio = None
        
        # Calculate row projected area on XY plane
        row_xy = np.asarray(row_pc.points)[:, :2]
        random_noise_row = np.random.uniform(-1e-4, 1e-4, size=(row_xy.shape[0], 1))
        row_points_3d = np.hstack([row_xy, np.full((row_xy.shape[0], 1), epsilon) + random_noise_row])
        pcd_row_xy = o3d.geometry.PointCloud()
        pcd_row_xy.points = o3d.utility.Vector3dVector(row_points_3d)
        
        try:
            row_proj_hull, _ = pcd_row_xy.compute_convex_hull()
            row_projected_area = row_proj_hull.get_surface_area()
        except Exception as e:
            print(f"Error computing projected area for row {row_id}:", e)
            row_projected_area = None
        
        # Calculate canopy area ratio
        if row_projected_area is not None and original_projected_area > 0:
            canopy_area_ratio = (row_projected_area * 3) / original_projected_area
        else:
            canopy_area_ratio = None
        
        # Collect results
        result = {
            "row_id": row_id,
            "num_points": pts.shape[0],
            "height_max": height_max,
            "height_min": height_min,
            "height_mean": height_mean,
            "plant_height": plant_height,
            "convex_hull_volume": convex_hull_volume,
            "minimal_cube_volume": minimal_cube_volume,
            "volume_ratio": volume_ratio,
            "row_projected_area": row_projected_area,
            "canopy_area_ratio": canopy_area_ratio,
            "obb_extent_x": dims[0],
            "obb_extent_y": dims[1],
            "obb_extent_z": dims[2]
        }
        all_results.append(result)
        print(f"Row {row_id} metrics:", result)
        
        # Visualize row with convex hull and OOBB
        try:
            hull, _ = row_pc.compute_convex_hull()
            hull_lineset = create_lineset_from_mesh(hull)
        except Exception as e:
            print(f"Error creating hull lineset for row {row_id}:", e)
            hull_lineset = None
        
        # Create OOBB lineset (blue)
        oobb_lineset = create_oobb_lineset(oobb)
        geometries = [row_pc]
        if hull_lineset is not None:
            geometries.append(hull_lineset)
        geometries.append(oobb_lineset)
        o3d.visualization.draw_geometries(geometries, window_name=f"Row {row_id} with Convex Hull and OOBB")
    
    # Visualize all rows with different colors
    o3d.visualization.draw_geometries(colored_rows, window_name="Three Major Rows Segmentation")
    
    return {
        "pcd_rotated": pcd_rotated,
        "soil_cloud_rotated": soil_cloud_rotated,
        "canopy_final": canopy_final,
        "major_rows": major_rows,
        "all_results": all_results
    }

if __name__ == "__main__":
    # Example usage
    file_path = "site_20230623_02.ply"  # Update with your file path
    # Specify planting direction if known, e.g. [1, 2.3]
    results = process_point_cloud(file_path, planting_direction=[1, 2.3])