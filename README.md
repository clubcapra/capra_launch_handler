
# capra_launch_handler

Package that allows the launch of .launch files using our web UI

## Usage

This package needs to be launched on robot startup to ensure its user friendliness. To do so, it uses Systemctl to start the launch service on boot which then executes a script that allows the launch of launch_handler.launch. This launch file launches the ros_bridge_server and the launch_handler nodes.

Both scripts needed to use this package are located in the scripts folder of this repository.

The system service should be placed in /lib/systemd/system/launch_script.service
The launch bash script should be placed in /usr/bin/launch_script.sh

**To enable the service**

`sudo systemctl enable launch_script.service`

**To disable the service**

`sudo systemctl disable launch_script.service`

**To start the service**

`sudo systemctl start launch_script.service`

**To stop the service**

`sudo systemctl stop launch_script.service`

## Launch file configuration

To add or remove launch files in the UI you can simple edit [launchFiles.ts](https://github.com/clubcapra/capra_web_ui/blob/master/src/renderer/store/modules/launchFiles.ts) in the capra_web_ui repository. Make sure to clear the cache once a modification is made.