# generic solution information
SolutionInfo info

# trajectory
moveit_msgs/RobotTrajectory trajectory

# planning scene of end state as diff w.r.t. start state
moveit_msgs/PlanningScene scene_diff

# list of controller names to use
std_msgs/String[] controller_names
