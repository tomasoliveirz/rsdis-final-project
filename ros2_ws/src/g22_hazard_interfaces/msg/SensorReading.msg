# Gas and radiation reading from one robot. Position in the map frame.

std_msgs/Header header
string robot                 # uav1, ugv1 or ugv2
geometry_msgs/Point position
float32 gas_ppm
float32 radiation_usvh       # microsievert per hour
