# NRE - Automating the LUCI Wheelchair 

## Running the Docker Container
> [!NOTE]  
> These steps are to be used on Linux (Ubuntu)
> Install Docker Desktop for Windows to use in WSL2

1. Clone this repository
2. `cd` into the root of this repo
3. Run `source ws_aliases.sh`
4. Run `ws-start` to start the image
5. Run `ws-exec` to open an interactive shell into the container


To stop using the shell, run `exit` while inside the container, and `ws-stop` to completely stop and remove the container

Example:
```bash
mkdir -p ~/git/nre_luci_ura
cd ~/git/nre_luci_ura
git clone git@github.com:NRE-Lab-UW/luci_wheelchair.git . # Will need to set up SSH keys
source ws-aliases.sh
ws-start
ws-exec
```
