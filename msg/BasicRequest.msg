# does nothing, but acknowledges
uint32 REQUEST_NOOP             = 0
# get the poses in generation order
uint32 REQUEST_GET_POSE_LIST    = 1
# get ordered pose indices, from the fittest (index 0)
uint32 REQUEST_GET_BEST_INDICES = 2
# get ordered poses, from the fittest (index 0)
uint32 REQUEST_GET_BEST_POSES   = 3
# get the weight (fitness) for each pose, in generation order
uint32 REQUEST_GET_POSE_WEIGHTS = 4
# get the number of poses
uint32 REQUEST_GET_POSE_COUNT   = 5
# returns the number of unknown voxel visible from each pose, in generation order
uint32 REQUEST_GET_UNK_COUNT    = 6

uint32 type

bool has_index_range
uint32 start_from_index
uint32 end_before_index
