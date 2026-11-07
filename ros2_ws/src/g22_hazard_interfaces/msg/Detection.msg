# A hazard found by the UAV (estimate) or checked on the ground by a UGV (confirmation).
# Position in the map frame.

std_msgs/Header header
uint32 id                    # same id for the estimate and its confirmation
string hazard_type           # gas or radiation
geometry_msgs/Point position
float32 value                # ppm for gas, uSv/h for radiation
string robot                 # robot that sent it
bool confirmed               # confirmations only: true if the ground reading is above the threshold
