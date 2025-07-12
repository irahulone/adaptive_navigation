#!/usr/bin/env bash


# Function to run service
systemd_stop(){

    sudo systemctl disable $1
    sudo systemctl stop $1
}

# List all service files in the services directory
mapfile -t SERVICE_FILES < <(cd services && ls *.service *.timer 2> /dev/null)
echo $SERVICE_FILES

# Options
declare -a options=(${SERVICE_FILES[@]} "all" "quit")

# Display numbered menu
echo "Choose one or more numbers (e.g., 1 3):"
for i in "${!options[@]}"; do
    echo "$((i + 1))) ${options[i]}"
done

# Prompt user input
read -p "#? " -a selections

# Process each selected number
for sel in "${selections[@]}"; do

    if [[ $sel = ${#options[@]} ]]; then

        echo Exiting...
        exit;

    elif [[ $sel = $((${#options[@]} - 1)) ]]; then

        echo Stopping all files...
        for file in ${SERVICE_FILES[@]}
        do
            systemd_stop $file
        done
        
    # Ensure it's a number and within range
    elif [[ "$sel" =~ ^[0-9]+$ ]] && (( sel >= 1 && sel <= $((${#options[@]} - 2)) )); then
        
        echo "Running option $sel with ${SERVICE_FILES[sel - 1]}"
        systemd_stop ${SERVICE_FILES[sel - 1]}

    else
        echo "Invalid selection: $sel"
    fi
done