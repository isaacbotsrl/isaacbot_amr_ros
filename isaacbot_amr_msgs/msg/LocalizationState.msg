# Localization mode managed by isaacbot_amr_indoor_nav_manager (latched: transient-local, depth 1).
# The manager picks it from the range sensors: a working lidar or camera localizes with AMCL
# on the selected map, or maps with SLAM when none is selected; with neither it dead-reckons.

uint8 UNAVAILABLE = 0       # No localization running
uint8 MAPPING = 1           # slam_toolbox is building a new map
uint8 LOCALIZATION = 2      # map_server + AMCL on a saved map
uint8 SWITCHING = 3         # Stopping one and starting the other
uint8 ODOMETRY = 4          # No lidar or camera: map -> odom held, dead-reckoning on odometry

# scan_source: the scan SLAM/AMCL read on localization/scan
uint8 SCAN_NONE = 0
uint8 SCAN_LIDAR = 1
uint8 SCAN_CAMERA = 2       # RealSense depth image as a LaserScan

std_msgs/Header header
uint8 mode
string map_name             # Map in use (LOCALIZATION, ODOMETRY) or being built (MAPPING, may be empty)
string message
uint8 scan_source
