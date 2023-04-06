source optitrack_setup.sh

export CONTROL_PARAMS=./custom_config/optitrack_hawk_manipulator_posctl_thrust.params.yaml
export OBJECT_NAME=hawk1
export ODOM_TOPIC=/$OBJECT_NAME/vrpn_client/estimated_odometry
