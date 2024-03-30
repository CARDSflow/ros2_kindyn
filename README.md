# ros2_common_utilities

This package is the ROS2 update of the [kindyn](https://github.com/CARDSflow/kindyn/tree/main) package. It contains the core c++ implementation for controlling cable robots.

## Usage
This package is used together with
* [iyntree](https://github.com/robotology/idyntree/tree/master)
* [qpOASES](https://github.com/robotology-dependencies/qpOASES/tree/master)
* [ros2_common_utilities](https://github.com/Roboy/ros2_common_utilities)
* [ros2_control_kindyn](https://github.com/CARDSflow/ros2_kindyn)
* [ros2_roboy3_models](https://github.com/Roboy/ros2_roboy3_models)
* [ros2_roboy_communication](https://github.com/Roboy/roboy_communication/tree/ros2_roboy3)

## Installation
clone these 6 packages inside a clean ros2_workspace.
```bash
    cd my/ros2_ws/src/
    rm -r log install build 
    git clone git@github.com:robotology/idyntree.git
    git clone git@github.com:robotology-dependencies/qpOASES.git
    git clone git@github.com:Roboy/ros2_common_utilities.git
    git clone git@github.com:CARDSflow/ros2_kindyn.git
    git clone git@github.com:Roboy/ros2_roboy3_models.git
    git clone --branch ros2_roboy3 git@github.com:Roboy/roboy_communication.git    
```

## Build
After these 6 packages are cloned into the ros2_workspace
change line 9 inside idyntree/src/core/include/iDynTree/EigenHelpers.h ```#include <Eigen/Dense>``` to ```#include <eigen3/Eigen/Dense>```
and build these packages in the following order:
```bash
    cd my/ros2_ws/
    source /opt/ros/humble/setup.zsh
    colcon build --packages-select ros2_roboy_communication
    source install/setup.zsh
    colcon build --packages-select ros2_common_utilities
    source install/setup.zsh
    colcon build --packages-select iDynTree qpOASES ros2_roboy3_models
    source install/setup.zsh
    colcon build --packages-select ros2_control_kindyn
    source install/setup.zsh
```

## Run
```bash
    cd my/ros2_ws/
    source /opt/ros/humble/setup.zsh
    source install/setup.zsh
    ros2 launch ros2_control_kindyn robody.launch.py
    ros2 launch ros2_control_kindyn send_cmd.launch.py
```
