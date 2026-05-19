# Modbus ROS2 Control

通用的 Modbus 末端执行器 ROS2 Control 硬件接口，支持多种使用 Modbus RTU 协议的末端执行器（夹爪等）。

## 架构设计

本包采用**抽象基类 + 具体实现**的设计模式，便于扩展支持不同的 Modbus 末端执行器协议：

```
ModbusGripperBase (抽象基类)
    ├── ChangingtekGripper (Changingtek 夹爪实现)
    └── [未来可扩展] OtherGripper (其他夹爪实现)
```

## Build

* Install libmodbus-dev
    ```bash
    sudo apt-get install libmodbus-dev 
    ```
* Build the package
    ```bash
    cd ~/ros2_ws
    colcon build --packages-up-to modbus_ros2_control --symlink-install
    ```

### 代码结构

代码按功能模块组织在子文件夹中：

```
modbus_ros2_control/
├── communicator/          # Modbus 通信层
│   └── ModbusRtuCommunicator
├── grippers/              # 夹爪相关（基类和实现）
│   ├── ModbusGripperBase (抽象基类)
│   └── ChangingtekGripper (具体实现)
└── ModbusHardware         # ROS2 Control 硬件接口（入口点）
```

### 核心组件

1. **communicator/ModbusRtuCommunicator** - Modbus RTU 通信封装类
    - 封装 libmodbus 的底层通信
    - 提供统一的读写接口

2. **grippers/ModbusGripperBase** - 夹爪抽象基类
    - 定义统一的接口（初始化、读取、写入）
    - 提供 ROS2 Control 接口导出功能
    - 位置归一化（0.0=闭合，1.0=打开）

3. **grippers/ChangingtekGripper** - Changingtek 夹爪具体实现
    - 实现 Changingtek 特定的 Modbus 协议
    - 位置范围：0-9000（0=打开，9000=闭合）

4. **hardware/ModbusHardware** - ROS2 Control 硬件接口
    - 管理 Modbus 连接
    - 创建和初始化夹爪对象
    - 实现 ROS2 Control 生命周期

## 使用方法

### 1. Changingtek 夹爪配置

#### 在 URDF 中配置

```xml
<ros2_control name="ModbusGripperSystem" type="system">
  <hardware>
    <plugin>modbus_ros2_control/ModbusHardware</plugin>
    <!-- 只需指定 gripper_type，其他参数会自动配置 -->
    <param name="gripper_type">changingtek</param>
    
    <!-- 可选：指定变体 (90c 或 90d) -->
    <param name="variant">90c</param>
    
    <!-- 如果需要覆盖默认值，可以显式指定 -->
    <!-- <param name="serial_port">/dev/ttyUSB1</param> -->
    <!-- <param name="slave_id">2</param> -->
  </hardware>
  
  <joint name="gripper_joint">
    <command_interface name="position"/>
    <state_interface name="position">
      <param name="initial_value">0.0</param>
    </state_interface>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
</ros2_control>
```

### 2. O7 灵巧手配置

#### 在 URDF 中配置

```xml
<ros2_control name="DexterousHandSystem" type="system">
  <hardware>
    <plugin>modbus_ros2_control/DexterousHandHardware</plugin>
    <!-- 串口路径 -->
    <param name="serial_port">/dev/ttyUSB0</param>
    <!-- 手部：left 或 right (默认: right) -->
    <param name="hand_side">right</param>
  </hardware>
  
  <!-- 7个关节：拇指弯曲、拇指横摆、食指、中指、无名指、小指、拇指横滚 -->
  <joint name="thumb_pitch">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
  
  <joint name="thumb_yaw">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
  
  <joint name="index_pitch">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
  
  <joint name="middle_pitch">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
  
  <joint name="ring_pitch">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
  
  <joint name="little_pitch">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
  
  <joint name="thumb_roll">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
</ros2_control>
```

#### 使用 Xacro 宏（推荐）

可以创建一个 xacro 文件来简化配置：

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  
  <xacro:macro name="o7_dexterous_hand_interfaces" params="name serial_port:=/dev/ttyUSB0 hand_side:=right">
    <ros2_control name="${name}_dexterous_hand" type="system">
      <hardware>
        <plugin>modbus_ros2_control/DexterousHandHardware</plugin>
        <param name="serial_port">${serial_port}</param>
        <param name="hand_side">${hand_side}</param>
      </hardware>
      
      <joint name="${name}_thumb_pitch">
        <command_interface name="position"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
      </joint>
      
      <joint name="${name}_thumb_yaw">
        <command_interface name="position"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
      </joint>
      
      <joint name="${name}_index_pitch">
        <command_interface name="position"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
      </joint>
      
      <joint name="${name}_middle_pitch">
        <command_interface name="position"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
      </joint>
      
      <joint name="${name}_ring_pitch">
        <command_interface name="position"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
      </joint>
      
      <joint name="${name}_little_pitch">
        <command_interface name="position"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
      </joint>
      
      <joint name="${name}_thumb_roll">
        <command_interface name="position"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
      </joint>
    </ros2_control>
  </xacro:macro>
  
</robot>
```

然后在主 URDF 中使用：

```xml
<xacro:include filename="$(find your_package)/xacro/o7_hand_interfaces.xacro"/>
<xacro:o7_dexterous_hand_interfaces name="left" serial_port="/dev/ttyUSB0" hand_side="left"/>
<xacro:o7_dexterous_hand_interfaces name="right" serial_port="/dev/ttyUSB1" hand_side="right"/>
```

### 3. 末端力矩/速度调试（ROS 参数）

夹爪（`ModbusHardware`）与灵巧手（`DexterousHandHardware`）均使用**统一**参数，不区分左右或单关节：

- `tool_torque`（double，0~1）
- `tool_velocity`（double，0~1）

位置仍由上层 controller 经 **position command interface** 下发（`target_percent` 等），不要用 param 设位置。

```bash
# 夹爪硬件节点（节点名以 launch 为准）
ros2 param set /<modbus_gripper_node> tool_torque 0.8
ros2 param set /<modbus_gripper_node> tool_velocity 1.0

# 灵巧手硬件节点（每只手一个 ros2_control 节点，各自一套 tool_*）
ros2 param set /linkerhand_7_left_system tool_torque 0.8
ros2 param set /linkerhand_7_left_system tool_velocity 1.0
```

### 4. 配置参数说明

#### Changingtek 夹爪参数

**必需参数**：
- `gripper_type`: 夹爪类型（"changingtek"）

**可选参数**：
- `variant`: 变体类型（"90c" 或 "90d"，默认 "90c"）
- `serial_port`: 串口路径（默认 `/dev/ttyUSB0`）
- `baudrate`: 波特率（默认 `115200`）
- `slave_id`: Modbus 从站地址（默认 `1`）
- `parity`: 校验位（默认 `N`，无校验）
- `data_bits`: 数据位（默认 `8`）
- `stop_bits`: 停止位（默认 `1`）

#### O7 灵巧手参数

**必需参数**：
- `serial_port`: 串口路径（如 `/dev/ttyUSB0`）

**可选参数**：
- `hand_side`: 手部类型（"left" 或 "right"，默认 "right"）— 仅用于 Modbus 从站地址，与 `tool_torque`/`tool_velocity` 无关
  - "left" → Modbus ID: 0x28 (40)
  - "right" → Modbus ID: 0x27 (39)
- `tool_torque`、`tool_velocity`: 该节点上**全部关节**共用同一力矩/速度（见上文）

**固定参数**（不可修改）：
- `baudrate`: `115200`
- `parity`: `N`（无校验）
- `data_bits`: `8`
- `stop_bits`: `1`

### 4. 位置单位

#### Changingtek 夹爪
- **ROS2 Control 接口**：归一化值 0.0-1.0
    - 0.0 = 完全闭合
    - 1.0 = 完全打开
- **Changingtek 协议**：0-9000（单位：mm）
    - 0 = 完全打开
    - 9000 = 完全闭合
- 转换由 `ChangingtekGripper` 自动处理

#### O7 灵巧手
- **ROS2 Control 接口**：归一化值 0.0-1.0
    - 0.0 = 最小位置（弯曲/靠拢）
    - 1.0 = 最大位置（伸直/远离）
- **Modbus 协议**：0-255
    - 0 = 最小位置
    - 255 = 最大位置
    - 128 = 中间位置
- 转换由 `SimpleDexterousHandWrapper` 自动处理

### 5. 启动方法

#### 使用 basic_joint_controller（推荐）

最简单的方法是使用 `basic_joint_controller` 包提供的 launch 文件：

```bash
# 启动左手（direction=1）
ros2 launch basic_joint_controller hand.launch.py \
    hand:=linkerhand \
    type:=o7 \
    hardware:=real \
    direction:=1 

# 启动右手（direction=-1）
ros2 launch basic_joint_controller hand.launch.py \
    hand:=linkerhand \
    type:=o7 \
    hardware:=real \
    direction:=-1 
```

该 launch 文件会自动处理所有必要的步骤，包括启动 ros2_control_node、加载控制器等。

#### 自定义 Launch 文件

如果需要自定义 launch 文件，可以参考 `basic_joint_controller/launch/hand.launch.py` 的实现。

关键步骤：
1. 使用 `linkerhand_description` 包的 xacro 文件
2. 设置 `ros2_control_hardware_type=real` 以使用 Modbus 硬件
3. 传递 `serial_port` 和 `direction` 参数给 xacro
4. 启动 `ros2_control_node` 和控制器

### 6. 快速启动示例

#### 使用 basic_joint_controller Launch 文件（推荐）

```bash
# 1. 编译
cd ~/ros2_ws
colcon build --packages-up-to modbus_ros2_control linkerhand_description basic_joint_controller --symlink-install
source install/setup.bash

# 2. 设置串口权限（选择以下方法之一）

## 方法1：临时设置（每次重启后需要重新设置）
sudo chmod 666 /dev/ttyUSB0  # 根据实际串口调整

## 方法2：永久设置 - 将用户添加到 dialout 组（推荐）
sudo usermod -a -G dialout $USER
# 然后注销并重新登录，或运行：newgrp dialout

## 方法3：永久设置 - 使用 udev 规则（最灵活）
# 安装 udev 规则文件
sudo cp $(find . -name "99-ttyusb-permissions.rules") /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
# 重新插拔 USB 设备使其生效

# 3. 启动灵巧手（左手，direction=1）
ros2 launch basic_joint_controller hand.launch.py \
    hand:=linkerhand \
    type:=o7 \
    hardware:=real \
    direction:=1 

# 或启动右手（direction=-1）
ros2 launch basic_joint_controller hand.launch.py \
    hand:=linkerhand \
    type:=o7 \
    hardware:=real \
    direction:=-1 
```

该 launch 文件会自动：
- 启动 `robot_state_publisher`
- 启动 `ros2_control_node`
- 加载 `joint_state_broadcaster` 控制器
- 加载 `hand_joint_controller` 控制器

#### 控制灵巧手

```bash
# 发送位置命令（归一化值 0.0-1.0）
# 格式：[thumb_joint1, thumb_joint2, thumb_joint3, index_joint, middle_joint, ring_joint, pinky_joint]
ros2 topic pub /hand_joint_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]}"

# 查看关节状态
ros2 topic echo /joint_states

# 查看控制器状态
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

## 扩展支持新的夹爪

要添加新的 Modbus 夹爪支持，只需：

1. **创建新的夹爪类**（继承 `ModbusGripperBase`）：

```cpp
class NewGripper : public ModbusGripperBase {
public:
    bool initialize(ModbusRtuCommunicator* communicator,
                    const std::map<std::string, std::string>& params) override;
    bool readStatus() override;
    bool writeCommand() override;
    void shutdown() override;
};
```

2. **在 `ModbusHardware::on_init()` 中添加创建逻辑**：

```cpp
if (gripper_type_ == "new_gripper") {
    gripper_ = std::make_unique<NewGripper>(...);
}
```

3. **实现协议特定的读写逻辑**

## 依赖

- `libmodbus-dev` - Modbus 通信库
- ROS2 Control 相关包
- `controller_manager` - 控制器管理
- `joint_state_broadcaster` - 关节状态广播
- `position_controllers` - 位置控制器（用于灵巧手控制）

## 安装依赖

```bash
sudo apt-get install libmodbus-dev
sudo apt-get install ros-${ROS_DISTRO}-controller-manager
sudo apt-get install ros-${ROS_DISTRO}-joint-state-broadcaster
sudo apt-get install ros-${ROS_DISTRO}-position-controllers
```

