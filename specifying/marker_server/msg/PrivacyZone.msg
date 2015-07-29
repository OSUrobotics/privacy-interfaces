string name  # descriptive name, used as unique id
string[] labels  # these should be used to look up filter levels
string owner

# Shorthands for field "status"
uint8 NOT_YET=0  # will become ACTIVE_NOW at time_begin (defined below)
uint8 ACTIVE_NOW=1  # privacy filter is in effect
uint8 SWITCHED_OFF=2  # user has (temporarily) disabled filtering for this zone
uint8 EXPIRED=3  # time_end has passed; filter no longer in effect

uint8 status

# time to start and stop filtering this zone
time time_begin
time time_end

Header header  # the tf frame in here is the basis for field "extents" 

shape_msgs/SolidPrimitive extents  # use "BOX" to define zone dimensions

