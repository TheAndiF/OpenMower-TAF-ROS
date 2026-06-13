int32 currentMowingPath
int32 currentMowingArea
int32 currentMowingPathIndex
string currentMowingPlanDigest
float32 currentMowingAngleIncrementSum
string currentMowingAreaId

# currentMowingPlan resume metadata.
# checkpoint.bag stays small and points to the saved plan snapshot.
string area_id
string area_digest
string plan_id
string plan_file
string current_path_id
int32 current_order
int32 current_pose_index
int32 last_completed_order
time saved_at

# Area queue / direct-area mowing metadata.
string mowing_area_mode
string area_queue_digest
int32 current_area_queue_index
