#!/bin/bash

# This script launches the mecanum controller node with the 'actual' driver type.

echo "Launching mecanum_controller with driver_type=actual..."
roslaunch mecanum_control mecanum_controller.launch driver_type:=actual