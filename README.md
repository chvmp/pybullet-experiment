## Installation

- Install [miniconda](https://docs.conda.io/en/latest/miniconda.html):
    ```
    cd /tmp
    wget https://repo.continuum.io/miniconda/Miniconda3-latest-Linux-x86_64.sh
    bash Miniconda3-latest-Linux-x86_64.sh  
    export PATH="$HOME/miniconda3/bin:$PATH"
    ```

- Create conda environment and install dependencies:

    ```
    . $HOME/miniconda3/etc/profile.d/conda.sh

    yes | conda create --name bullet --channel conda-forge \
    ros-core \
    ros-actionlib \
    ros-dynamic-reconfigure \
    
    conda activate bullet

    yes | pip install pybullet --upgrade --user
    git clone https:github.com/chvmp/pybullet-experiment

    ```


## Quickstart

- Run CHAMP's controller:

    ```
    roslaunch champ_config bringup.launch has_imu:=false
    ```

- on another terminal:

    ```
    roslaunch champ_teleop teleop.launch
    ```

- Running the simulation:

    ```
    cd pybullet-experiment/scripts
    python sim.py
    ```