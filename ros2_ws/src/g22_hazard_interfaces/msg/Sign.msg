# Hazard sign seen by a camera. Only a hint that there might be a leak.
# Position in the map frame.

uint8 GAS=0
uint8 RADIATION=1

std_msgs/Header header
uint32 id
uint8 type
geometry_msgs/Point position
string seen_by
