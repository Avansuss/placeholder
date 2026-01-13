# Status Constants
int8 UNKNOWN=0
int8 IDLE=1
int8 MOVING=2
int8 WAITING_ELEVATOR=3
int8 CHARGING=4
int8 ERROR=99

# Current state
int8 status
string description  # Bijv: "Onderweg naar keuken"

# Location Data
string last_visited_node
string target_node

# Vitality
float32 battery_level
