#!/bin/bash
# Take ownership of persistent data folders
sudo chown -R $USER:$USER ~/.cache
sudo chown -R $USER:$USER ~/.local
sudo chown -R $USER:$USER ~/.nv
sudo chown -R $USER:$USER ~/.nvidia-omniverse

# Take ownership of isaac-sim folders
sudo chown $USER:$USER /isaac-sim/kit
sudo chown -R $USER:$USER /isaac-sim/kit/logs
sudo chown $USER:$USER /isaac-sim/exts
sudo chown -R $USER:$USER /isaac-sim/exts/omni.isaac.synthetic_recorder

# Configure omniverse cache 
cp ./.devcontainer/omniverse.toml /home/vscode/.nvidia-omniverse/config/omniverse.toml

# Run the postinstall script for isaac sim
# Check to see if post install has already been run
if [ ! -d ~/.cache/ov/Kit ]; then
    echo "ISAAC SIM POST INSTALL RUNNING... THIS WILL TAKE ABOUT 10min"
    bash /isaac-sim/omni.isaac.sim.post.install.sh
fi                                                                                                                                                                                                                                                                                                      
