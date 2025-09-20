# CanopyPC: Plant Canopy Point Cloud Processing Tool

A Python-based tool for processing and analyzing UAV-CCO (Unmanned Aerial Vehicle Cross-Circular Oblique) reconstructed plant canopy point clouds. This tool provides a complete pipeline for segmenting, analyzing, and visualizing 3D point cloud data of plant canopies.

## Features

- **Ground-Canopy Segmentation**: Separate ground and plant canopy points using Cloth Simulation Filter (CSF)
- **Point Cloud Preprocessing**: 
  - Noise removal using statistical outlier detection
  - Small cluster removal using DBSCAN
  - Landmark sign removal
  - Manual point selection and removal
- **Canopy Row Segmentation**: Automatically segment plant canopies into rows using K-means clustering
- **Geometric Analysis**:
  - Convex hull volume calculation
  - Oriented bounding box (OOBB) analysis
  - Projected area calculation
  - Plant height statistics
- **Visualization**: Interactive 3D visualization of point clouds, convex hulls, and bounding boxes

## Requirements

- Python 3.6+
- Open3D
- NumPy
- Matplotlib
- scikit-learn
- SciPy
- Shapely
- CSF (Cloth Simulation Filter)

## Installation

1. Clone this repository:
   ```
   git clone https://github.com/smiler488/canopypc.git
   cd canopypc
   ```

2. Install dependencies using requirements.txt:
   ```
   pip install -r requirements.txt
   ```

   Or install dependencies manually:
   ```
   pip install open3d numpy matplotlib scikit-learn scipy shapely CSF
   ```

## Usage

### Basic Usage

```python
import process_point_cloud as ppc

# Process a point cloud file
results = ppc.process_point_cloud("path/to/your/pointcloud.ply")
```

### Specifying Planting Direction

If you know the planting direction, you can specify it:

```python
results = ppc.process_point_cloud("path/to/your/pointcloud.ply", 
                                 planting_direction=[1, 2.3])
```

### Processing Pipeline

1. **Load Point Cloud**: The tool loads a .ply format point cloud file
2. **Ground Segmentation**: CSF algorithm separates ground and non-ground points
3. **Point Cloud Correction**: Rotates and translates the point cloud to align with coordinate axes
4. **Noise Removal**: Multiple filtering steps to clean the canopy point cloud
5. **Row Segmentation**: Segments the canopy into rows based on planting direction
6. **Metrics Calculation**: Calculates various geometric metrics for each row
7. **Visualization**: Displays the results at each processing stage

## Output Metrics

For each segmented row, the tool calculates:

- **Point Count**: Number of points in the row
- **Height Statistics**: Maximum, minimum, mean height, and plant height (max-min)
- **Volume Metrics**: Convex hull volume and volume ratio
- **Area Metrics**: Projected area and canopy area ratio
- **Dimensional Metrics**: OOBB extents in X, Y, and Z directions

## Interactive Features

- **Manual Point Removal**: Select and remove unwanted points or clusters
- **3D Visualization**: Visualize point clouds, convex hulls, and bounding boxes

## Example

```python
from process_point_cloud import process_point_cloud

# Process a sample point cloud
results = process_point_cloud("site_20230623_02.ply")

# Access the results
soil_cloud = results["soil_cloud_rotated"]
canopy_cloud = results["canopy_final"]
rows = results["major_rows"]
metrics = results["all_results"]

# Print metrics for each row
for row_metrics in metrics:
    print(f"Row {row_metrics['row_id']} height: {row_metrics['plant_height']:.2f}")
    print(f"Row {row_metrics['row_id']} volume: {row_metrics['convex_hull_volume']:.2f}")
```

## Technical Details

### CSF Algorithm

The Cloth Simulation Filter (CSF) algorithm is used for ground segmentation. It simulates a cloth dropping onto an inverted point cloud to separate ground and non-ground points.

### Row Segmentation

Row segmentation uses the following approach:
1. Determine the planting direction (user-specified or calculated using PCA)
2. Project points onto an axis perpendicular to the planting direction
3. Apply K-means clustering to separate rows
4. Order rows based on their position along the separation axis

### Volume Calculation

Convex hull volume is calculated using the divergence theorem on the triangular mesh of the convex hull. For accurate volume calculation, the bottom surface is filled with additional points.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Citation

If you use this tool in your research, please cite:

```
@software{CanopyPC,
  author = {Liangchao Deng},
  title = {CanopyPC: Plant Canopy Point Cloud Processing Tool},
  year = {2025},
  url = {https://github.com/smiler488/canopypc}
}
```

## Acknowledgments

- The CSF algorithm implementation is based on [the work by Zhang et al.](https://www.mdpi.com/2072-4292/8/6/501)
- Open3D library for point cloud processing