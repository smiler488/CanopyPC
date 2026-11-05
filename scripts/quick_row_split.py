"""
Quick look at a plot point cloud without CSF.

This is the exploratory script I used before the CSF pipeline in
``process_point_cloud.py`` was in place. It is kept because it is handy for
a fast sanity check of a new reconstruction:

1. statistical outlier removal and centring;
2. minimum-area rectangle (OpenCV) and PCA alignment for a visual check;
3. ground removal by a height percentile;
4. equal-width split into N rows along X, with height and convex-hull
   volume printed per row; the rows are written as ``canopy_row_<i>.ply``.

The percentile ground cut and the equal-width split are crude on purpose.
Use ``process_point_cloud.py`` for numbers that go into a paper.

Example
-------
    python scripts/quick_row_split.py site.ply --ground-percentile 10 --rows 3 --out-dir out/

Author: Liangchao Deng (邓良超)
License: MIT
"""

import argparse
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d
from scipy.spatial import ConvexHull


def mbr_lineset(pcd):
    """3-D box built on the minimum-area XY rectangle, spanning the z range."""
    pts = np.asarray(pcd.points)
    rect = cv2.minAreaRect(pts[:, :2].astype(np.float32))
    box = cv2.boxPoints(rect)
    corners = np.zeros((8, 3))
    corners[:4, :2] = box
    corners[4:, :2] = box
    corners[:4, 2] = pts[:, 2].min()
    corners[4:, 2] = pts[:, 2].max()
    lines = [[0, 1], [1, 2], [2, 3], [3, 0],
             [4, 5], [5, 6], [6, 7], [7, 4],
             [0, 4], [1, 5], [2, 6], [3, 7]]
    ls = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(corners),
                              lines=o3d.utility.Vector2iVector(lines))
    ls.paint_uniform_color([1, 0, 0])
    return ls


def pca_align(pcd):
    """Rotate the cloud so that its principal axes match X, Y, Z (in place)."""
    pts = np.asarray(pcd.points)
    eigval, eigvec = np.linalg.eigh(np.cov(pts.T))
    eigvec = eigvec[:, np.argsort(eigval)[::-1]]
    if np.linalg.det(eigvec) < 0:  # keep a right-handed frame
        eigvec[:, 2] *= -1
    pcd.rotate(eigvec.T, center=pcd.get_center())
    return pcd


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("input", help="input .ply")
    ap.add_argument("--rows", type=int, default=3)
    ap.add_argument("--ground-percentile", type=float, default=10.0,
                    help="points below this z percentile are treated as ground (default: 10)")
    ap.add_argument("--out-dir", default=".", help="where to write canopy_row_<i>.ply")
    ap.add_argument("--no-vis", action="store_true")
    args = ap.parse_args(argv)

    show = (lambda g: None) if args.no_vis else o3d.visualization.draw_geometries
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)

    pcd = o3d.io.read_point_cloud(args.input)
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    pcd.translate(-pcd.get_center())
    show([pcd, mbr_lineset(pcd), axis])

    pca_align(pcd)
    obb = pcd.get_oriented_bounding_box()
    obb.color = (1, 0, 0)
    show([pcd, obb, axis])

    pts = np.asarray(pcd.points)
    canopy = pts[pts[:, 2] > np.percentile(pts[:, 2], args.ground_percentile)]

    edges = np.linspace(canopy[:, 0].min(), canopy[:, 0].max(), args.rows + 1)
    idx = np.clip(np.digitize(canopy[:, 0], edges[1:-1]), 0, args.rows - 1)

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    row_clouds = []
    for i in range(args.rows):
        row = canopy[idx == i]
        if row.shape[0] < 4:
            print(f"Row {i + 1}: too few points")
            continue
        volume = ConvexHull(row).volume
        print(f"Row {i + 1}: points={row.shape[0]}, max z={row[:, 2].max():.3f}, "
              f"hull volume={volume:.4f}")
        rc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(row))
        o3d.io.write_point_cloud(str(out_dir / f"canopy_row_{i + 1}.ply"), rc)
        row_clouds.append(rc)
    show(row_clouds)


if __name__ == "__main__":
    main()
