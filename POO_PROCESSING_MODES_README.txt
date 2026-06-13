OpenMower POO processing modes and executable path metadata
===========================================================

This package adds an explicit Path Order Optimizer processing mode and stores
all executable path transformations in currentMowingPlan.

Parameters
----------

path_order_optimizer_processing_mode:
  1 = slicer_order
      Keep the original slicer path order. No POO reordering.

  2 = ordered_fills_plus_ordered_obstacles
      Area outlines -> ordered fill paths -> ordered obstacle outlines.
      This is the default mode.

  3 = ordered_obstacles_plus_ordered_fills
      Area outlines -> ordered obstacle outlines -> ordered fill paths.

  4 = mixed_fills_and_obstacles
      Area outlines -> fill paths and obstacle outlines ordered together.

path_order_optimizer_optimize_outer_outline_entry:
  true/false. When true, the first closed outer area outline is rotated in the
  executable path so that it starts at the best reachable outline point from the
  current mower pose. The original slicer path remains unchanged.

Current mowing plan rules
-------------------------

slicer_source.path:
  Original slicer geometry. Never reordered, reversed or rotated.

execution.path:
  Fully prepared executable path. This is the only path used for movement.
  If a path is reversed or the outer area outline is rotated, the execution path
  already contains the transformed point order and recomputed pose orientations.

execution.rotation_offset:
  0 means no rotation. A value > 0 means execution.path starts at the original
  slicer_source.path pose with this index. Currently used for the optimized outer
  area outline entry.

execution.transform_flags:
  Human-readable debug/status flags. Current values:
    reversed
    rotated_outer_outline_entry

Build-log note
--------------

The last uploaded build log failed in open_mower/settings_persistence.h because
it used nlohmann::ordered_json through packages that did not receive the vendored
nlohmann target include path. The helper now uses nlohmann::json in that header,
so it no longer depends on ordered_json being available through every including
package.
