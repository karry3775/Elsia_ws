### About

This repository contains code for a proof of concept localization method
that uses structural cues in a man-made environment (warehouses for our case) and generates
state estimates that are drift free in a local sense. <br/>

We call this method
**ELSIA**  which is short for **E**xploitative **L**ocalization using **S**tructure inside **A**isles.

## Dependencies
- keyboard module python
    - ``` sudo su && pip install keyboard```

### Codes to run
- Spawn robot in an environment
    - ```$ roslaunch elsia jacky_spawn.launch```
- Open up the teleoperation code (this needs to be done in sudo mode as per the requirements of keyboard module in python)
    - ```$ cd && sudo su ```
    - ```$ source Documents/Elsia_ws/devel/setup.bash && rosrun elsia jacky_teleoperate.py```