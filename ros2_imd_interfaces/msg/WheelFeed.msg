std_msgs/Header header

# transform matrix(Tx, Ty, Ttheta)
float64[3] mat
# pose [m]
float32 pose
# velocity [m/sec]
float32 velocity
# error [m/sec] (= feedback-command)
float32 velocity_error