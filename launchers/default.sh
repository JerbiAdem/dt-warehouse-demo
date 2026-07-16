#!/bin/bash

source /environment.sh
dt-launchfile-init

dt-exec roslaunch warehouse warehouse.launch veh:="$VEHICLE_NAME"

dt-launchfile-join
