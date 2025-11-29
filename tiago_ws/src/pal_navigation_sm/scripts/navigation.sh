#! /bin/sh
#
# Check if the $HOME/.pal folder is available and creates it if needed before
# launching navigation.
#
# Usage: $0 <robot> [<state>] [<localization method>] [<mapping method>] [<map>]

# Check parameters:
if [ $# -lt 1 ]; then
  echo "Usage: $0 <robot> [<state>] [<localization method>] [<mapping method>] [<map>] [<scan_topic>] [<laser_model>]"
  echo "Check if the $HOME/.pal folder is availabe and creates it if needed"
  echo "before launching navigation."
  exit 1
else
  ROBOT=$1
fi

if [ $# -lt 2 ]; then
  STATE=localization
else
  STATE=$2
fi

if [ $# -lt 3 ]; then
  LOCALIZATION=amcl
else
  LOCALIZATION=$3
fi

if [ $# -lt 4 ]; then
  MAPPING=gmapping
else
  MAPPING=$4
fi

if [ $# -lt 5 ]; then
  MAP=$HOME/.pal/${ROBOT}_maps/config
else
  MAP=$5
fi

if [ $# -lt 6 ]; then
  SCAN_TOPIC=rgbd_scan
else
  SCAN_TOPIC=$6
fi

if [ $# -lt 7 ]; then
  LASER_MODEL=false
else
  LASER_MODEL=$7
fi

if [ $# -lt 9 ]; then
  MULTI="false"
  ROBOT_NAMESPACE=""
else 
  if [ "$9" = "true" ]; then
    if [ $# -lt 10 ]; then
      echo "If MULTI is true I need the robot_namespace"
      exit 1
    else
      MULTI="true"
      ROBOT_NAMESPACE=$10
    fi
  else 
    MULTI="false"
    ROBOT_NAMESPACE=""
  fi
fi

# Ensure target directory exists
if [ ! -d "$MAP" ]; then
  echo "Warning: Target path is not a directory: $MAP."

  PKG=`echo $MAP | awk '{ n=split($0, s, "/"); for (i=1; i<=n; i++) { if (index(s[i], "_maps") != 0) { print s[i] } } }'`
  rosrun pal_navigation_sm cp_maps_to_home.sh $PKG
  if [ $? -ne 0 ]; then
    echo "Error: Failed to copy maps from $PKG to $HOME/.pal."
    exit 3
  fi
fi

# Ensure pose file exists
if [ ! -f "$HOME/.pal/pose.yaml" ]; then
  rosrun pal_navigation_sm cp_pose_to_home.sh
  if [ $? -ne 0 ]; then
    echo "Error: Failed to copy pose to $HOME/.pal."
    exit 3
  fi
fi

# Add / to namespace if not empty
if [ -n "$NS" ]; then
  NS="$NS"/
fi

# Run localization/mapping
roslaunch ${ROBOT}_2dnav_gazebo $STATE.launch localization:=$LOCALIZATION mapping:=$MAPPING map:=$MAP multiple:=$MULTI robot_namespace:=$ROBOT_NAMESPACE scan_topic:=$SCAN_TOPIC laser_model:=$LASER_MODEL

