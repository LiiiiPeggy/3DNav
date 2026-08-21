# Spiral assets

Spiral is referenced by the PCT paper but comes from the 3D2M planner project:
<https://github.com/ZJU-FAST-Lab/3D2M-planner>.

That repository does not currently state an explicit license.  To avoid
redistributing material without a clear grant, `spiral0.3_2.pcd`, its generated
tomogram, and its derived route are ignored by Git.  Prepare them locally:

```bash
./scripts/fetch_spiral_asset.sh
./scripts/setup_tomography_env.sh
./scripts/generate_spiral_tomogram.sh
./scripts/test_pct_spiral_astar.sh --force
```

Local verified checksums:

```text
24acbdccc1bb743cf698cfbb5dd739aa6253977cd22dc9c089c4e3d9e36248ff  spiral0.3_2.pcd
9d5267926a340cce6f5b486c56f9573e9d7a64de058df40da23807e153dbded7  spiral0.3_2_ros2.pickle
```

The default video route is `(-16, -6, 0)` to `(3.2, -22.2, 4.0)`: 38.30 m,
3.80 m elevation gain, two tomogram layers, and 0.40 m 5-percentile clearance.
