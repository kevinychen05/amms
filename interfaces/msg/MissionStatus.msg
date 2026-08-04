# Published continuously by mission_executive_node to report current mission state.

std_msgs/Header header
# header.stamp is the time this status was generated.

string mission_id
# Unique identifier for the current mission run, assigned when a MissionCommand goal
# is accepted. Empty string when idle.

string state
# Current state name, e.g. "IDLE", "NAVIGATE", "SEARCH", "DETECT", "ALIGN", "GRASP",
# "TRANSPORT", "PLACE", "COMPLETE", "FAILED". Kept as a string rather than an enum/int
# so log files and RViz displays stay human-readable without a lookup table.

string previous_state
# The state transitioned from, empty string on the first status of a mission.

uint32 retry_count
# Number of retries attempted at the current state, for states with retry/backoff logic.

string last_error
# Human-readable description of the most recent failure. Empty string if none.
