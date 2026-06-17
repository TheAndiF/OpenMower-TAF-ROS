# mowing_path_order_optimizer

Optional stage between `slic3r_coverage_planner` and `mower_logic`.

The node provides:

```text
/mowing_path_order_optimizer/optimize_paths
```

It receives the raw slicer paths and returns prepared execution paths with stable source identities, execution order, optional reverse direction, optional outer-outline entry rotation and transformation metadata.

## Disabled behavior

When `path_order_optimizer_enabled` is false, the optimizer behaves as pass-through and returns the original slicer order unchanged.

## Processing modes

`path_order_optimizer_processing_mode` selects the path grouping/order strategy:

```text
1 = slicer_order
2 = ordered_fills_plus_ordered_obstacles
3 = ordered_obstacles_plus_ordered_fills
4 = mixed_fills_and_obstacles
```

Area outlines are kept as the first group for the grouped and mixed optimizer modes. Fill and obstacle paths are then ordered according to the selected mode.

## Outline entry modes

`path_order_optimizer_outline_entry_mode` controls how the area-outline block is entered:

```text
0 = slicer_entry
1 = approach_inner_outline_entry
```

Mode `1` first plans the approach from the current mower pose to the original start point of the innermost closed area outline. The first intersection between that approach path and the innermost outline is used to calculate an entry angle relative to the outer area centroid. All closed area outlines are rotated synchronously to that angle. After the rotated outline block has been appended, the end point of the rotated outline sequence is used as the start pose for ordering the remaining fill and obstacle paths. The original slicer path is not modified; the rotation is reported via `rotation_offsets` and `transform_flags`.

## Reverse handling

When `path_order_optimizer_allow_reverse` is true, paths can be selected in reverse direction if their end point is cheaper to reach. Reversed execution paths are explicitly reversed and their orientations are recalculated before being returned.

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
