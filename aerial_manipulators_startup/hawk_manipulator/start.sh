#!/bin/bash

# Session input argument. Defaults to session.yml if not provided
SESSION=${2:-session.yml}
arr_split=(${SESSION//./ })
extension=${arr_split[${#arr_split[@]}-1]}

if [ "$extension" == "yml" ] || [ "$extension" == "yaml" ]; then
  :
else
  SESSION="session.yml"
fi

# Absolute path to this script. /home/user/bin/foo.sh
SCRIPT=$(readlink -f $0)
# Absolute path this script is in. /home/user/bin
SCRIPTPATH=`dirname $SCRIPT`
cd "$SCRIPTPATH"

# remove the old link
rm .tmuxinator.yml

# link the session file to .tmuxinator.yml
ln $SESSION .tmuxinator.yml

SETUP_NAME=$1
[ -z "$SETUP_NAME" ] && SETUP_NAME=optitrack_setup.sh

# start tmuxinator
tmuxinator uav_ros_indoor setup_name=$SETUP_NAME

