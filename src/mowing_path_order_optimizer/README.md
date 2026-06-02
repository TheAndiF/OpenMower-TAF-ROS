# mowing_path_order_optimizer

Optional stage between `slic3r_coverage_planner` and `mower_logic`.

The node provides:

```text
/mowing_path_order_optimizer/optimize_paths
```

It receives the raw slicer paths and either returns them unchanged or reorders them.

## Disabled behavior

When `path_order_optimizer_enabled` is false, the optimizer behaves as pass-through and returns the original slicer order unchanged.

## Enabled behavior

The intended group order is:

```text
1. area outlines
2. fill paths
3. obstacle outlines
```

Fill paths can be reordered greedily. For each remaining fill path the optimizer evaluates the transition to its start point and, when allowed, its end point. If the end point is cheaper, the path is reversed and orientations are recalculated.

## Cost modes

```text
0 = euclidean
1 = planner
2 = hybrid
```

`hybrid` first preselects candidates using euclidean distance and then evaluates the nearest candidates with the MBF `GetPath` action.

## Fallbacks

The node is designed to fail open:

- planner failure can fall back to euclidean cost
- optimizer failure can fall back to slicer order
- mower_logic can continue with the slicer result if the optimizer service is unavailable

