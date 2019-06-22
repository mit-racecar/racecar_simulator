# This message contains all possible sensor information in one message

Header header
# LaserScan
sensor_msgs/LaserScan laserscan
# Odometry
nav_msgs/Odometry odom
# collision boolean and collision direction
bool collision
uint32 collision_ind
# top down image
sensor_msgs/Image img

# future sensors
# imu, wheel_rpm?