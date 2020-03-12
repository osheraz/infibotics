# infibotics

## package includes implementation of various nav stacks & control for OPTIMUS robot






Clone the repo, build (make) everything


**Simulation of basic Infibotics plant**

```roslaunch infi_sim_v1 main.launch```

***params:***

arm       - includes arm

gazebo    - start simulation in gazebo env

joy       - includes joystick control

lidar     - include lidar node

move_base - planner utilities     (navigation)

gmapping  - slam navgiation option (navigation)

```roslaunch infi_sim_v1 main.launch gmapping:=true move_base:=true```

amcl      - slam navgiation asyn ml localization / requires map (navigation)

```roslaunch infi_sim_v1 main.launch amcl:=true move_base:=true```

hector    - slam navgiation hector slam method (navigation)

```roslaunch infi_sim_v1 main.launch hector:=true move_base:=true```

carto     - slam navgiation cartographer method (navigation)

```roslaunch infi_sim_v1 main.launch carto:=true move_base:=true```



