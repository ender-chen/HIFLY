# Any of the channels may not be available and be set to NaN
# to indicate that it does not contain valid data.
# The variable names follow the definition of the
# MANUAL_CONTROL mavlink message.
# The default range is from -1 to 1 (mavlink message -1000 to 1000)
# The range for the z variable is defined from 0 to 1. (The z field of
# the MANUAL_CONTROL mavlink message is defined from -1000 to 1000)
uint64 timestamp

float32 x			 # stick position in x direction -1..1
				 # in general corresponds to forward/back motion or pitch of vehicle,
				 # in general a positive value means forward or negative pitch and
				 # a negative value means backward or positive pitch
float32 y			 # stick position in y direction -1..1
				 # in general corresponds to right/left motion or roll of vehicle,
				 # in general a positive value means right or positive roll and
				 # a negative value means left or negative roll
float32 z			 # throttle stick position 0..1
				 # in general corresponds to up/down motion or thrust of vehicle,
				 # in general the value corresponds to the demanded throttle by the user,
				 # if the input is used for setting the setpoint of a vertical position
				 # controller any value > 0.5 means up and any value < 0.5 means down
float32 r			 # yaw stick/twist position, -1..1
				 # in general corresponds to the righthand rotation around the vertical
				 # (downwards) axis of the vehicle

