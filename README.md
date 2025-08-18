
# Steps

## On the Raspberry Pi (Source)
1. Run the following command to start the source
```bash
# If connected to real hardware 
ros2 run rf_scalar_field rf_source

# If running on simulation
ros2 run rf_scalar_field rf_source --ros-args -r __ns:=/sim
```
2. To change the power level (between 0 (weakest) to 4 (strongest)) of the antenna source, on a new terminal, run the following to get the current power level
```bash
# If on hardware, get the current power level (default 0)
ros2 param get /rf_source power_level

# If on simulation (default 0)
ros2 param get /sim/rf_source power_level
```
while setting the correct power level
```bash
# If on hardware, get the current power level (default 0)
ros2 param set /rf_source power_level 1 # Between 0 and 4

# If on simulation (default 0)
ros2 param set /sim/rf_source power_level 1 # Between 0 and 4

```

## On the Computer or Pioneer (Receiver)
1. Run the following to receive the GPS and receiver 
- Prerequisities: Must have the `multi_robots` repo. If running on a VIM you may need to run `export BLINKA_FORCECHIP=BCM2XXX`. Then run `python3 -c "import adafruit_gps; print('success')"` 
```bash
# Run the following GPS
ros2 run gps_core run_gps1

# Turn on the receiver
ros2 run rf_scalar_field rf_receiver

```
- If running in simulation, run the following
```bash
ros2 launch rf_scalar_field receiver.launch.py ns:=/sim

```
2. Run the following to subscribe to the data 
```bash

# If on hardware
ros2 run synchronizer collect_gps_rssi --ros-args -p gps_sub_topic:=/None/gps1


# If on simulation
ros2 run synchronizer collect_gps_rssi --ros-args -r __ns:=/sim

```
3. (Archived) Run the following for logging and live plotting
```bash
ros2 launch data_insights launch.py # Append the ns:=/sim if in sim
```

3B. Alternatively, run the following
```bash
ros2 run reference_srv gps_reference_server 

ros2 run convert_pose converter --ros-args -p robot_id:=None -p disable_imu:=True

 ros2 run synchronizer collect_gps_rssi --ros-args -p gps_sub_topic:=/None/gps1

ros2 run data_insights live_contour_plotter 
```