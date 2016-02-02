uint8 GF_ACTION_NONE = 0                    # no action on geofence violation
uint8 GF_ACTION_WARN = 1                    # critical mavlink message
uint8 GF_ACTION_LOITER = 2                  # switch to AUTO|LOITER
uint8 GF_ACTION_RTL = 3                     # switch to AUTO|RTL
uint8 GF_ACTION_TERMINATE = 4               # flight termination

uint8 RESTRICTED_AREA_WARNING_NONE = 0		# restricted area safe
uint8 RESTRICTED_AREA_WARNING_APPROACH = 1	# restricted area approach
uint8 RESTRICTED_AREA_WARNING_VIOLATED = 2	# restricted area violated

bool geofence_violated		# true if the geofence is violated
uint8 geofence_action       # action to take when geofence is violated
uint8 restricted_area_warning	# warning of restricted area
