# RASBerry Data Collection

## Database Setup

Please refer to [docs/database.md](database.md) for the schema. 

Please run the `setup_database.sh` script located in the `scripts` folder to create the SSD mount rules and directories. Note you will need to change this script to reflect the hardware you will be collecting data on, currently it's configured for the three SSDs bought by the Data Collection fund.

```bash
roscd rasberry_data_collection/scripts
# Only run this once per system since multiple runs will break the fstab
./setup_database.sh
```

Install MongoDB >=3.2 on your system. Or use the installation script in `scripts/install_mongodb.sh` for `4.0.1`. If `ros-kinetic-mongodb-store` there will be a conflict, the only solution currently is to install over the top and remove the mongodb-store package.

```bash
roscd rasberry_data_collection/scripts
./install_mongodb.sh
```

## Odroid Setup

Please run the `odroid_setup.sh` script located in the `scripts` folder to set i2c permissions.

```bash
roscd rasberry_data_collection/src/ip_camera
./odroid_setup.sh
```

## Cameras Setup

The cameras purchased by the data collection project are currently used in the scenario files (ex: [scripts/scenarios/inter_row_robot19.yaml](../scripts/scenarios/inter_row_robot19.yaml)).
Most issues had with the cameras can be resolved by resetting them through the `realsense-viewer` software and loading the base config file to them [config/base.json](../config/base.json).

You can find the serial numbers of connected devices via:

```bash
rs-enumerate-devices | grep "  Serial Number"
```

## Installation

Please checkout the branch used in the [RASBerry/#362](https://github.com/LCAS/RASberry/pull/362) PR 
(or the master if merged) in your RASBerry source folder.

### Simulation

Create a file located at `~/.rasberryrc` containing the following:

```bash
ROBOT_NAME="thorvald_019"
SCENARIO_NAME="sim_riseholme_poly_act"
```

Launch the autonomous data collection bring up scripts:

```bash
roscd rasberry_bringup/tmule

# Terminate any existing tmule sessions
tmule -c rasberry-simple_robot_corner_hokuyos_autonomous_data_capture.yaml -W 3 terminate

# Launch the tmule session
tmule -c rasberry-simple_robot_corner_hokuyos_autonomous_data_capture.yaml -W 3 launch
```

Follow the standard instructions

### Autonomous

Run the autonomous data collection launch file and pass the scenario you want to run.

```bash 
roslaunch rasberry_data_collection run_scenario.launch scenario_file:="<scenario_file_here>"
```