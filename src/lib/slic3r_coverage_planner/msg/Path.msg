uint8 TYPE_FILL=0
uint8 TYPE_AREA_OUTLINE=1
uint8 TYPE_OBSTACLE_OUTLINE=2

# Kept for backward compatibility with existing consumers.
uint8 is_outline

# Explicit path classification so downstream order optimizers can distinguish
# outer area outlines from inner obstacle outlines.
uint8 path_type

nav_msgs/Path path
