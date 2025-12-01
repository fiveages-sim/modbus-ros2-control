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

### 1. 在 URDF 中配置

```xml
<ros2_control name="ModbusGripperSystem" type="system">
  <hardware>
    <plugin>modbus_ros2_control/ModbusHardware</plugin>
    <!-- 只需指定 gripper_type，其他参数会自动配置 -->
    <param name="gripper_type">changingtek</param>
    
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

### 2. 配置参数说明

#### 必需参数
- `gripper_type`: 夹爪类型（当前支持 "changingtek"）
  - 根据此参数会自动配置默认的 Modbus 参数

#### 可选参数（会根据 `gripper_type` 自动设置默认值，可手动覆盖）

**Changingtek 夹爪默认值**：
- `serial_port`: `/dev/ttyUSB0`
- `baudrate`: `115200`
- `slave_id`: `1`
- `parity`: `N`（无校验）
- `data_bits`: `8`
- `stop_bits`: `1`

如果您的配置与默认值不同，可以在 URDF 中显式指定这些参数来覆盖默认值。

### 3. 位置单位

- **ROS2 Control 接口**：归一化值 0.0-1.0
  - 0.0 = 完全闭合
  - 1.0 = 完全打开

- **Changingtek 协议**：0-9000（单位：mm）
  - 0 = 完全打开
  - 9000 = 完全闭合

转换由 `ChangingtekGripper` 自动处理。

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

## 安装依赖

```bash
sudo apt-get install libmodbus-dev
```

