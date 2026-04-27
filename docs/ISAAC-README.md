# Usage / 使用方法

## English

### Enable Extension

Open Windows > Extensions, search for "zmq.bridge", and enable it.

### Quick Start

1. Create an OmniGraph.
2. Add a ZmqPublish* or ZmqSubscribe* node from the ZmqBridge category.
3. Set `pubAddress` or `subAddress` and `topicName`.
4. Connect `execIn` to a tick or action trigger.

### Notes

- Supported Data Types: Float(Array), Double(Array), Int(Array), Twist (CmdVel), JointState, Clock, Image, Imu, Nav/GPS, Depth/Altitude.
- ZMQ topics are simple strings; keep them consistent with your ROS2 side.
- Ensure inputs/outputs include `description` fields in .ogn definitions.
- PUB/SUB is best-effort; avoid blocking calls in node compute.

## 中文

### 启用扩展

打开 Windows > Extensions，搜索 "zmq.bridge" 并启用。

### 快速开始

1. 创建 OmniGraph。
2. 在 ZmqBridge 分类下添加需要发布的节点（如 `ZmqPublishFloat`, `ZmqPublishFloatArray` 等）。
3. 设置 `pubAddress` / `subAddress` 和 `topicName`。
4. 将 `execIn` 连接到 tick 或 action 触发器。

### 备注

- 支持的数据类型：Float(Array), Double(Array), Int(Array), Twist (CmdVel), JointState, Clock, Image, Imu, Nav/GPS, 深度计/高度计 (Depth/Altitude).
- ZMQ 话题为字符串，需与 ROS2 侧保持一致。
- .ogn 定义中的输入/输出必须包含 `description` 字段。
- PUB/SUB 为尽力而为传输，节点计算中不要阻塞。

### 订阅协议 (Subscription Protocol)

**自动订阅**: ZmqSubscribe* 节点首次执行时会自动向 ROS2 bridge 发送订阅请求。

**控制话题**: `__zmq_bridge_control__` - 用于 Isaac Sim 告知 ROS2 bridge 需要订阅的话题。

**详细文档**: 参见 [INTERFACE.md](INTERFACE.md) 中的 Subscription Protocol 章节。

---

## Documentation

- [INTERFACE.md](INTERFACE.md) - Complete API reference and architecture
- [CHANGELOG.md](CHANGELOG.md) - Version history
