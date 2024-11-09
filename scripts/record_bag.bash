get_name_date()
{
  eval name="$1"
  eval current_date=$(date '+%Y_%m_%d-%H_%M_%S')
}
get_name_date "\${name}"
name=$1
name_date="$current_date"_"$name"

rosbag record -O $name_date /leica/position /leica/position/rostime /prism/pose /imu/data_raw /graph_msf/opt_odometry_world_imu /graph_msf/est_odometry_world_imu /camera/odom/sample /leica/state /tf_static /tf -e "/vrpn_clien(.*)/estimated_odometry"
