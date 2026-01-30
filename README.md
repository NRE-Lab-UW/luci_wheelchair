# NRE - Automating the LUCI Wheelchair

## Running the Docker Container

> [!NOTE]  
> These steps are to be used on Linux (Ubuntu)
> Install Docker Desktop for Windows to use in WSL2

1. Clone this repository
2. `cd` into the root of this repo
3. Run `source ws_aliases.sh` (or add this line to your shell profile (bashrc) so every new terminal loads the aliases automatically)
4. Run `ws-up` to start the image FROM THE ROOT OF THE REPO
5. Run `ws-exec` to open an interactive shell into the container, you should be able to run this from anywhere on your system

To stop using the shell, run `exit` while inside the container, and `ws-down` to completely stop and remove the container

Example:

```bash
mkdir -p ~/git/nre_luci_ura
cd ~/git/nre_luci_ura
git clone git@github.com:NRE-Lab-UW/luci_wheelchair.git . # Will need to set up SSH keys
source ws_aliases.sh
ws-up
ws-exec
```


## Wheelchair SDK


Connect to the wheelchair: `ros2 run luci_grpc_interface grpc_interface_node -a 10.0.0.21`

See all topics: `ros2 topic list`

Relevant topics:
- `/luci/speed_setting` returns animal speed profile from 1-5 (TODO: Find out how this modifies overall kinematic profile)
- `/luci/chair_profile` returns chair speed profile from 1-5 (TODO: Find out how this modifies overall kinematic profile)
- `/luci/odom` returns pose, orientation, and twist of the wheelchair
- `/luci/joystick_position` returns messages published from remote joystick nodes

### Notes

- Containers are started with a fixed name (`luci_ros2_sdk`). If you already have a running container for `luci-ros2-sdk` with a different name, `ws-exec` will still find it and attach.
- Add `source /path/to/ws_aliases.sh` to your `~/.bashrc` or `~/.zshrc` to avoid sourcing manually in every terminal.


TODO: Test launch file and write here how to run it - might need to move them into packages?
colcon build