
## 1. ROS2 genaral
```bash
# Load ROS2 setup and workspace setup
source /opt/ros/humble/setup.bash
source install/setup.bash

# 查看节点列表
ros2 node list

# 查看话题列表
ros2 topic list

# 查看服务列表
ros2 service list

# 查看节点信息
ros2 node info /hand_right/hand_right_node

# 查看topic publish/subscribe 详情
ros2 topic info "some topic name" --verbose

# 查看service的具体type
ros2 service type /hand_right/get_temp

# 查看消息/服务/动作接口完整结构
ros2 interface show 接口全名
```

### 2. PnC specific

```
colcon build --symlink-install
colcon build --packages-select carla_ros_bridge
colcon build --packages-select carla_shenlan_mpc_controller --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug

ros2 launch carla_ros_bridge carla_ros_bridge.launch.py
ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py
ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py synchronous_mode:=false
```

cd /home/pe_user/PnC/carla_ros2_ws
ros2 launch carla_shenlan_bridge_ego_vis carla_bridge_ego_vehilce.launch.py

ros2 run carla_shenlan_pid_controller carla_shenlan_pid_controller_node
ros2 run carla_shenlan_stanley_pid_controller carla_shenlan_stanley_pid_controller_node
ros2 launch carla_shenlan_lqr_pid_controller lqr_launch.py
ros2 launch carla_shenlan_mpc_controller mpc_launch.py
ros2 launch carla_shenlan_mpc_controller mpc_launch.py debug:=true
rviz2 -d src/carla-ros-bridge/carla_shenlan_projects/carla_shenlan_mpc_controller/rviz/mpc_vis.rviz

bash src/carla-ros-bridge/carla_shenlan_projects/carla_shenlan_mpc_controller/scripts/debug_mpc.sh


cd ~/PnC/MPC_experiment
source /opt/ros/humble/setup.bash
source ~/PnC/carla_ros2_ws/install/setup.bash
source install/setup.bash
ros2 launch mpc_control mpc_control.launch.py

