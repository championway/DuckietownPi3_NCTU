#!/usr/bin/env bash
echo "Setting VEHICLE_NAME..."
if [ $# -gt 0 ]; then
	# provided a hostname, use it as ROS_MASTER_URI
	export VEHICLE_NAME=$1
	export PATROLLING_NUMBER=$1
else
	echo "No hostname provided. Using $HOSTNAME."
	export VEHICLE_NAME=$HOSTNAME
	export PATROLLING_NUMBER=4
fi
echo "VEHICLE_NAME set to $VEHICLE_NAME"
