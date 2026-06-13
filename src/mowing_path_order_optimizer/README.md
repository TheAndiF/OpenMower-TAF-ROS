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

`path_order_optimizer_outline_entry_mode` controls the entry point of the outer area outline:

```text
0 = slicer_entry
1 = nearest_outer_outline_entry
```

Mode `1` rotates the executable outer area outline so that it starts at the nearest/best reachable point from the current mower pose. The original slicer path is not modified; the rotation is reported via `rotation_offsets` and `transform_flags`.

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
