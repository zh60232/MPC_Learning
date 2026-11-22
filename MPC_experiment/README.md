# MPC Experiment

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

```bash
ros2 launch mpc_control mpc_control.launch.py
```

build:
colcon build --packages-select mpc_control --cmake-args -DCMAKE_BUILD_TYPE=Debug


MPC 使用当前车速和 0.01 秒模型步长沿参考线采样未来 10 步，并插值生成各步的参考状态；
到达轨迹末端后保持最后的参考位置和朝向。车辆静止时不向前推进预瞄距离。
