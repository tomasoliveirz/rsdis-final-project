# Hazard found by the UAV. Position in the map frame.

uint8 GAS=0
uint8 RADIATION=1

std_msgs/Header header
uint32 id
uint8 type
geometry_msgs/Point position
float32 value                # reading that found it, ppm or uSv/h
string found_by
