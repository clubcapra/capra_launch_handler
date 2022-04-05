
# capra_launch_handler

Package that allows the launch of .launch files using our web UI

## Usage

This package needs to be launched on robot startup to ensure its user friendliness. To do so, it uses Systemctl to start the launch service on boot which then executes a script that allows the launch of launch_handler.launch. This launch file launches the ros_bridge_server and the launch_handler nodes.

The system service is located in /lib/systemd/system/launch_script.service

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

## Limitations

Currently, killing files is not 100% reliable on every launch file. This is due to the fact that some launch files send arguments to other launch files which is not currently supported by the kill command. The launch files in ros_astra_camera are a good example of this problem. This is a limitation that will be fixed in the future.