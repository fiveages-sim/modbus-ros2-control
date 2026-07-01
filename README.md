# Modbus ROS2 Control

通用的 Modbus / RS485 末端执行器 ROS2 Control 硬件接口，支持夹爪与多种灵巧手。

## 1. 支持的末端执行器

本包注册 **5 个** `hardware_interface::SystemInterface` 插件（见 `modbus_ros2_control.xml`）：

| 插件 | 产品 | 识别 / 配置方式 |
|------|------|-----------------|
| **`ModbusHardware`** | Changingtek AG2F90 | `gripper_type:=changingtek`，`variant:=90c` / `90d` |
| **`DexterousHandHardware`** | LinkerHand **O7** | URDF 7 关节 |
| | LinkerHand **O6** | URDF 6 关节（默认） |
| | LinkerHand **L6** | `hand_type` 含 `L6` |
| | Freedom / Inspire | **不支持** — 初始化失败，请用对应专用插件 |
| **`InspireHandHardware`** | Inspire **RH56 系列**（E2 / F2） | URDF 6 关节；Modbus RTU（FC03/FC10）；限位写死为 RH56E2 |
| **`FreedomRS485Hardware`** | Freedom **V1** / **V2** | `protocol_version:=auto` / `freedomv1` / `freedomv2`（或按关节数推断） |
| **`XHand1RS485Hardware`** | **XHand1** | URDF 12 关节；专用 RS485（默认 3 Mbps） |
| **`TheoHandModbusHardware`** | TheoHand **STD16A** | URDF 16 关节；标准 Modbus RTU（默认 115200 8N1，slave_id 1） |

**Inspire 请使用 `InspireHandHardware`**

- `hand_type` 含 `INSPIRE` 或 `RH56` 时，`DexterousHandHardware` 会拒绝初始化。
- USB 路径请用 `inspire_description` 的 `inspire_rs485_side_system` 宏（见 [§3.3](#33-inspire-rh56inspirehandhardware)）。

推荐在机器人 description 包中通过 xacro 宏接线，而非手写完整关节块（见 [§3 配置参考](#3-配置参考)）。

## 2. 代码结构

```
modbus_ros2_control/
├── communicator/
│   └── ModbusRtuCommunicator          # libmodbus 封装
├── grippers/
│   ├── ModbusGripperBase
│   └── ChangingtekGripper             # ModbusHardware
├── hands/
│   ├── linkerhand/                    # DexterousHandHardware
│   ├── inspire/                       # InspireHandHardware
│   ├── freedom/                       # FreedomRS485Hardware
│   ├── theo/                          # TheoHandModbusHardware
│   └── xhand1/                        # XHand1RS485Hardware
├── modbus_hardware.cpp                # 夹爪插件入口
└── modbus_ros2_control.xml            # 插件清单
```

夹爪路径复用 `gripper_hardware_common`（Changingtek、LinkerHand `DexterousHandHardware`）。`InspireHandHardware` 协议逻辑在包内自包含，与 `marvin_ros2_control` 并行实现（见 [§9 TODO](#9-todo)）。

## 3. 配置参考

关节接口定义放在各 `*_description` 包；此处仅示硬件插件与关键参数。完整关节列表请 include 对应 xacro。

### 3.1 Changingtek 夹爪（`ModbusHardware`）

```xml
<ros2_control name="ModbusGripperSystem" type="system">
  <hardware>
    <plugin>modbus_ros2_control/ModbusHardware</plugin>
    <param name="gripper_type">changingtek</param>
    <param name="variant">90c</param>
    <!-- 可选: serial_port, slave_id, baudrate -->
  </hardware>
  <joint name="gripper_joint">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
</ros2_control>
```

### 3.2 LinkerHand（`DexterousHandHardware`）

```xml
<ros2_control name="linkerhand_right_system" type="system">
  <hardware>
    <plugin>modbus_ros2_control/DexterousHandHardware</plugin>
    <param name="serial_port">/dev/ttyUSB0</param>
    <param name="hand_side">right</param>
    <param name="hand_type">linkerhand_o7</param>
  </hardware>
  <!-- include linkerhand_description xacro 宏，如 linkerhand_o7_interfaces -->
</ros2_control>
```

参考：`linkerhand_description/xacro/ros2_control/hand.xacro`（单臂 launch）、各机器人 `robot.xacro` 中的 `side_usb_hand_system`。

### 3.3 Inspire RH56（`InspireHandHardware`）

```xml
<!-- 推荐直接使用 inspire_description 宏 -->
<xacro:inspire_rs485_side_system side="left" serial_port="/dev/ttyUSB0" hand_type="inspire_f2"/>
```

宏定义：`inspire_description/xacro/ros2_control/side_systems.xacro`  
单臂示例：`inspire_description/xacro/ros2_control/hand.xacro`

### 3.4 Freedom / XHand1

```xml
<xacro:freedom_rs485_side_system side="left" serial_port="/dev/ttyUSB0" type="freedomv2"/>
<xacro:xhand1_rs485_side_system side="left" serial_port="/dev/ttyUSB1"/>
```

宏定义：`freedom_description`、`xhand1_description` 的 `xacro/ros2_control/side_systems.xacro`。

### 3.5 TheoHand STD16A（`TheoHandModbusHardware`）

```xml
<ros2_control name="theohand_system" type="system">
  <hardware>
    <plugin>modbus_ros2_control/TheoHandModbusHardware</plugin>
    <param name="serial_port">/dev/ttyUSB0</param>
    <param name="baudrate">115200</param>
    <param name="slave_id">1</param>
  </hardware>
  <!-- include theohand_description/xacro/ros2_control/std16a.xacro -->
</ros2_control>
```

插件按官方 `wn_hand_sdk-develop_modbus` demo 在激活时写控制字 `0x0f`。STD16A 当前按传入
`slave_id` 直接通信，不在激活阶段读取左右手寄存器；右手测试默认使用地址 `1`。

## 4. 硬件参数

| 插件 | 参数 | 默认 | 说明 |
|------|------|------|------|
| **ModbusHardware** | `gripper_type` | `changingtek` | 必需 |
| | `variant` | `90c` | `90c` / `90d` |
| | `serial_port` | `/dev/ttyUSB0` | |
| | `slave_id` | `1` | |
| | `baudrate` | `115200` | |
| **DexterousHandHardware** | `serial_port` | — | 必需 |
| | `hand_side` | `right` | `left` / `right`；左 `0x28`，右 `0x27` |
| | `hand_type` | `simple_dexterous_hand` | 区分 O6 / L6；含 `INSPIRE` / `RH56` 会拒绝 |
| | `max_speed_ratio` | — | 可选，如 `1.0` |
| | 串口格式 | 固定 | 115200 8N1 |
| **InspireHandHardware** | `serial_port` | `/dev/ttyUSB0` | |
| | `hand_side` | `left` | `slave_id:=auto` 时左 `2`、右 `1` |
| | `slave_id` | `auto` | |
| | `baudrate` | `115200` | |
| | `read_feedback` | `true` | |
| **FreedomRS485Hardware** | `protocol_version` | `auto` | `freedomv1` / `freedomv2` |
| | `serial_port` | `/dev/ttyUSB0` | |
| | `hand_side` | — | |
| | `command_speed` | `100` | |
| **XHand1RS485Hardware** | `serial_port` | `/dev/ttyUSB0` | |
| | `baudrate` | `3000000` | |
| | `hand_id` / `host_id` | `0` / `0xFE` | |
| **TheoHandModbusHardware** | `serial_port` | `/dev/ttyUSB0` | |
| | `baudrate` | `115200` | 协议 6.1：RS485 115200 8N1 |
| | `slave_id` | `1` | 可按设备地址覆盖 |
| | `read_feedback` | `false` | 需要实际位置反馈时可打开，读取 `0x0051` 起的 16 个实际位置 |
| | `background_period_ms` | `20` | 会在第一次 read 时跟随控制周期 |

## 5. 位置单位

| 末端 | ROS2 Control 接口 | 设备协议 |
|------|-------------------|----------|
| Changingtek 夹爪 | 0.0（闭合）~ 1.0（打开） | 0–9000（0=开，9000=合） |
| LinkerHand O6/O7 | 0.0 ~ 1.0（弯曲→伸直） | 0–255 |
| Inspire RH56 | 弧度（rad） | 寄存器原始值约 500–1750 |
| Freedom / XHand1 | 弧度（rad） | 各协议自定义映射 |
| TheoHand STD16A | 弧度 / 米（按 URDF 关节限位归一化） | 0–9000（协议位置单位约等于 0.01 deg） |

## 6. 编译与依赖

**系统依赖**

```bash
sudo apt-get install libmodbus-dev
sudo apt-get install ros-${ROS_DISTRO}-controller-manager \
                     ros-${ROS_DISTRO}-joint-state-broadcaster \
                     ros-${ROS_DISTRO}-position-controllers
```

**编译**

```bash
cd ~/ros2_ws
colcon build --packages-up-to modbus_ros2_control --symlink-install
source install/setup.bash
```

**ROS 包依赖**：`hardware_interface`、`pluginlib`、`rclcpp`、`rclcpp_lifecycle`、`gripper_hardware_common`、`arms_controller_common`。

## 7. 启动示例

以 LinkerHand O7 独立调试为例（`basic_joint_controller` + `linkerhand_description`）：

```bash
# 串口权限（任选其一）
sudo usermod -a -G dialout $USER && newgrp dialout
# 或: sudo chmod 666 /dev/ttyUSB0

# 启动
ros2 launch basic_joint_controller hand.launch.py \
  hand:=linkerhand type:=o7 hardware:=real direction:=1

# 发令 / 查看状态
ros2 topic pub /hand_joint_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]}"
ros2 topic echo /joint_states
ros2 control list_controllers
```

Inspire 单臂：`inspire_description` + `hand.launch.py`（`hardware:=real`）。双臂整机见各机器人 description 的 `robot.xacro` 与 launch。

自定义 launch 要点：xacro 生成 URDF → `ros2_control_node` → 加载 `joint_state_broadcaster` 与位置控制器。可参考 `basic_joint_controller/launch/hand.launch.py`。

## 8. 扩展新夹爪

1. 继承 `ModbusGripperBase` 实现协议读写。
2. 在 `ModbusHardware::on_init()` 中按 `gripper_type` 实例化。
3. 在 `modbus_ros2_control.xml` 中注册（若为新 `SystemInterface` 则另建插件类）。

## 9. TODO

- [ ] **抽取 Inspire 协议公共层** — `InspireHandHardware` 与 `marvin_ros2_control::InspireHandE2`（及 CAN FD 变体）各自实现同一套寄存器协议，未复用 `gripper_hardware_common`。后续抽到 common 供 Marvin / modbus / can 共用。
- [ ] **RH56F2 限位** — `InspireHandHardware` 关节上限仍写死为 RH56E2；F2 真机需按型号区分或从 URDF 读取。
