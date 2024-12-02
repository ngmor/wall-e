#
# Servo position as percentage of servo movement range
#

# Servo indexes
int32 HEAD_CENTER=0
int32 NECK_TOP=1
int32 NECK_BOTTOM=2
int32 EYE_RIGHT=3
int32 EYE_LEFT=4
int32 ARM_LEFT=5
int32 ARM_RIGHT=6

# Servo name (lower case of enumeration above)
string name

# Servo index (from enumeration above)
int32 index -1

# Servo position (0 to 100% of servo movement range as 0.0 to 1.0)
float64 position