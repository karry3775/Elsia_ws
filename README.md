### About

This repository contains code for a proof of concept localization method
that uses structural cues in a man-made environment (warehouses for our case) and generates
state estimates that are drift free in a local sense. <br/>

We call this method
**ELSIA**  which is short for **E**xploitative **L**ocalization using **S**tructure **I**nside **A**isles.

## Cloning the repo with submodules
```git clone --recurse-submodules git@github.com:karry3775/Elsia_ws.git```

## Building the repo
```catkin build -j8``` (Use -j flag to specify how many cores you want to use) 

## Environment setup
- Copy the environment models inside ```~/.gazebo/models```

## Dependencies
- keyboard module python
    - ``` sudo su && pip install keyboard```

### Codes to run
- Spawn robot in an environment
    - ```roslaunch elsia jacky_spawn.launch world_name:=isolated_aisles``` (OR)
    - ```roslaunch elsia jacky_spawn.launch world_name:=multiple_aisles``` (OR)
    - ```roslaunch elsia jacky_spawn.launch world_name:=warehouse_like```
- Open up the teleoperation code (this needs to be done in sudo mode as per the requirements of keyboard module in python)
    - ```cd && sudo su ```
    - ```source Documents/Elsia_ws/devel/setup.bash && rosrun elsia jacky_teleoperate.py```
- Initiate Block Counting
    - ```rosrun elsia block_counting.py```
