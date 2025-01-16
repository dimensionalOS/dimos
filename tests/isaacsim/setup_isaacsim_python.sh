#!/bin/bash

sudo apt install python3.10-venv
python3.10 -m venv env_isaacsim
source env_isaacsim/bin/activate

# Install pip packages 
pip install isaacsim==4.2.0.2 --extra-index-url https://pypi.nvidia.com
pip install isaacsim-extscache-physics==4.2.0.2
pip install isaacsim-extscache-kit==4.2.0.2
pip install isaacsim-extscache-kit-sdk==4.2.0.2 --extra-index-url https://pypi.nvidia.com

export OMNI_KIT_ACCEPT_EULA=YES

