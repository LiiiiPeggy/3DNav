# Building assets

This directory contains the matched geometry and traversability inputs for the
RViz-only Building demo:

- `pcd/building2_9.pcd`: the source point cloud consumed by SCAN's static map.
- `tomogram/building2_9_ros2.pickle`: the 0.10 m, eight-layer PCT tomogram generated
  from the same point cloud.
- `routes/ground_to_upper.csv`: a generated cross-floor A* smoke-test result.

The PCD originates from the official GPL-licensed PCT planner resources.  The
`_ros2` pickle is normalized so it can be loaded by Ubuntu 22.04's system NumPy
without importing an external PCT virtual environment.

The upstream project and attribution are documented at
<https://github.com/byangw/PCT_planner>.  If normalizing another upstream
pickle, run `scripts/normalize_pct_pickle.py --input INPUT --output OUTPUT`.

Expected SHA-256 checksums:

```text
fe7b55db5b41169fca2afb0d12065468849c7ccd06affffbdeed6c6a9b5480fe  building2_9.pcd
e0c6b950f50b4097f266cf69282b0ba64f1c46ebce04fa3a44a9d18197a4ece1  building2_9_ros2.pickle
```

The default start `(5, 5, 0)` and goal `(-6, -1, 14)` are snapped to valid
tomogram cells and deliberately exercise a multi-floor route. Both assets use
the `world` frame directly, so no coordinate transform is applied.
