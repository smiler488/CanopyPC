# CanopyPC

[English](README.md) | 中文

[![CI](https://github.com/smiler488/CanopyPC/actions/workflows/test.yml/badge.svg)](https://github.com/smiler488/CanopyPC/actions/workflows/test.yml) [![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE) ![Python 3.9–3.12](https://img.shields.io/badge/Python_3.9%E2%80%933.12-informational)
CanopyPC 把一个行播作物小区的无人机点云，转成按行统计的冠层性状表：株高、凸包体积、投影面积和包围盒尺寸。

这个工具是我在处理无人机交叉环绕倾斜（CCO）航拍重建的田间小区时写的。每个小区有几行，同一个试验里每个小区都要算同一套结构性状，不想再一个个在 CloudCompare 里手动量。我的点云来自摄影测量重建，其他带米制坐标的 `.ply` 也可以用。

## 处理流程

1. **去地面**：布料模拟滤波 CSF（Zhang et al., 2016）。
2. **调平**：对地面点做 RANSAC 平面拟合，把法向量旋到 +Z，并把地面平移到 z = 0，这样 z 就是离地高度。
3. **去噪**：统计离群点剔除、DBSCAN 去小簇、去掉小区标牌这类紧凑物体。可选弹出窗口手动框选删除残留。
4. **分行**：把点投影到垂直于播种方向的轴上，用一维 K-means 分行。播种方向可以手动给，也可以用 PCA 自动估计。
5. **按行计算性状**并导出 CSV。

| 列名 | 含义 |
|---|---|
| `num_points` | 该行点数 |
| `height_max`、`height_min`、`height_mean` | 相对地面平面的 z 统计量 |
| `plant_height` | `height_max - height_min` |
| `convex_hull_volume` | 在最低点封底后的凸包体积 |
| `minimal_cube_volume` | 以有向包围盒最长边为边长的立方体体积 |
| `volume_ratio` | `convex_hull_volume / minimal_cube_volume`，紧凑度指标 |
| `row_projected_area` | 该行在 XY 平面上二维凸包的面积 |
| `canopy_area_ratio` | `row_projected_area * 行数 / 整个小区投影面积` |
| `obb_extent_x/y/z` | 有向包围盒三边长 |

单位与输入点云一致。

## 安装

Python 3.9–3.12（Open3D 对新版 Python 的支持通常会晚一些）。

```bash
git clone https://github.com/smiler488/CanopyPC.git
cd CanopyPC
pip install -r requirements.txt
```

`CSF` 模块来自 PyPI 上的 `cloth-simulation-filter` 包。PyPI 上名叫 `CSF` 的包是另一个东西，不要装错。

## 使用

命令行：

```bash
python process_point_cloud.py plot.ply --rows 3 --direction 1 2.3 -o plot_traits.csv
```

常用参数：

- `--no-vis`：不弹任何窗口，适合批处理或服务器。
- `--no-manual`：保留可视化，但跳过手动清理。
- `--cloth-resolution`：CSF 布料网格大小。我的小区用 0.1 m 就够；地面起伏大或有坡度时调大一些。

在 Python 里调用：

```python
from process_point_cloud import process_point_cloud, write_results_csv

res = process_point_cloud("plot.ply", planting_direction=[1, 2.3], n_rows=3,
                          visualize=False, interactive=False)
write_results_csv(res["all_results"], "plot_traits.csv")
```

`scripts/quick_row_split.py` 是更早、更粗糙的版本（按高度分位数去地面、按 X 等宽切行），拿到新的重建结果想快速看一眼时还挺方便，所以留着。

## 示例数据

`data/sample/` 里是同一个小区的三行点云（`canopy_row_*.ply`）和该小区的 CSF 布料节点（`cloth_nodes.txt`），足够试一下性状计算函数：

```python
import numpy as np
import open3d as o3d
from process_point_cloud import row_traits, projected_area_xy

row = o3d.io.read_point_cloud("data/sample/canopy_row_1.ply")
plot_area = projected_area_xy(np.asarray(row.points)) * 3
traits, _ = row_traits(0, row, plot_area, n_rows=3)
print(traits)
```

## 注意事项

- K-means 分行默认各行宽度相近、行间有明显空隙。封行以后分行会变得不可靠，用数据之前先看一眼分行结果。
- 凸包体积会高估真实冠层体积，冠层越稀疏越明显。我把它当作处理间比较的相对指标，而不是生物量的绝对代理。需要绝对值时，alpha shape 或体素计数更合适。
- 标牌过滤会删掉凸包表面积小于 0.2（点云单位²）的紧凑簇，植株很小时要调低。

## 本次修订

- 投影面积之前约为真实值的 2 倍，已修正。
- 拟合出的地面法向量朝下时点云会被上下翻转，已修正。
- `canopy_area_ratio` 改为按实际行数计算，不再写死为 3。
- 新增命令行、CSV 导出和关闭可视化的开关。
- 依赖从错误的 `CSF` 改为 `cloth-simulation-filter`。

## 引用

请按 [`CITATION.cff`](CITATION.cff) 引用，或使用：

```bibtex
@software{deng_canopypc,
  author = {Deng, Liangchao},
  title  = {CanopyPC: row-level canopy traits from UAV point clouds},
  year   = {2025},
  url    = {https://github.com/smiler488/CanopyPC}
}
```

CSF：Zhang, W. et al. (2016). An easy-to-use airborne LiDAR data filtering method based on cloth simulation. *Remote Sensing*, 8(6), 501.

## 许可证

MIT，见 [LICENSE](LICENSE)。邓良超（Liangchao Deng）。
