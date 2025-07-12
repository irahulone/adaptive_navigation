#!/usr/bin/env bash

# Help bar
HELP=$(
cat <<'HEREDOC'
    Build the exec and service files and begin running the ros services on Linux Operating System
    Flags:
        -p = Ask for prompt
        -h = Display this help bar
        -v = Verbose
HEREDOC
)

# Variables
SYSTEMD_PATH=/etc/systemd/system
ROS_WS=robot_ws
AUTO_ACCEPT=1
VERBOSE=0

# Handle user arguments
while getopts :vph option; do
	case $option in
        v) VERBOSE=1;;
		p) AUTO_ACCEPT=0;;
        h) echo "$HELP"; exit;;
	esac
done

# Get absolute directory of the project itself
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# List all service files in the services directory
declare -a SERVICE_FILES="$(cd services && ls *.service *.timer 2> /dev/null)"

# Echo files
if [[ $VERBOSE -eq 1 ]]; then 
    echo $SERVICE_FILES
fi

# Move all the exec files to /opt
# declare -a EXEC_FILES="$(cd $PROJECT_DIR/exec && ls *.sh *.bash 2> /dev/null)"
sudo cp exec/* /opt

# Make dirs for systemd
sudo mkdir -p $SYSTEMD_PATH


# Replace HOME and USER environment variables
for f in ${SERVICE_FILES[@]}
do
    envsubst < $PROJECT_DIR/services/$f | sudo tee "$SYSTEMD_PATH/$f" > /dev/null
done

# Colcon build
if [[ $AUTO_ACCEPT -eq 1 ]]; then
    cd $HOME/$ROS_WS ; colcon build; cd -;
else
    read -e -p "Colcon build and Source? [Y/n] " -i "Y" answer
    echo $answer
    if [[ $answer = Y ]]; then
        cd $HOME/$ROS_WS ; colcon build; cd -;
    fi
fi

# Run services
read -e -p "Run services? [Y/n] " -i "Y" answer
echo $answer
if [[ $answer = Y ]]; then
    bash $PROJECT_DIR/scripts/run.sh
fi

