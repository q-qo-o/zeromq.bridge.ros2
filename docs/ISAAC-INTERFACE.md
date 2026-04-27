# Interface / 接口说明

## English

### ZMQ Context Management

The ZeroMQ Bridge uses a centralized **ZmqManager** singleton for lifecycle and resource management with a **unified addressing model**.

#### Unified Addressing Model

All data (clock, joint state, twist, camera images, etc.) is transmitted through a **single pair of global addresses**:

- **Publisher (PUB)**: `tcp://*:25556` - One address for all published data
- **Subscriber (SUB)**: `tcp://127.0.0.1:25557` - One address for all subscribed data

Different data types are distinguished by **topic names** in each message.

#### Default Addresses

If no addresses are configured, the system uses defaults:

- **Publisher (PUB)**: `tcp://*:25556`
- **Subscriber (SUB)**: `tcp://127.0.0.1:25557`

#### Node: ZmqContext

The **ZmqContext** node must be placed at the start of your action graph to initialize the ZMQ global addresses.

**Inputs**:

- `execIn` (execution): Trigger input
- `pubAddress` (string): Global ZMQ PUB bind address. Default: `tcp://*:25556`
- `subAddress` (string): Global ZMQ SUB connect address. Default: `tcp://127.0.0.1:25557`
- `lingerMs` (int): Socket linger time for graceful shutdown. Default: `100ms`

**Outputs**:

- `execOut` (execution): Trigger output for chaining to other nodes
- `outPubAddress` (string): Current global publisher address
- `outSubAddress` (string): Current global subscriber address

#### Global Address Management

Once ZmqContext is executed:

- All subsequent **publish nodes** automatically use the global PUB address
- All subsequent **subscribe nodes** automatically use the global SUB address
- Nodes can still override the global address by providing their own `pubAddress`/`subAddress` input (optional)

```
ZmqContext sets: global_pub = "tcp://*:25556", global_sub = "tcp://127.0.0.1:25557"
                 ↓
All other nodes use these addresses automatically (socket pooling prevents duplication)
                 ↓
Data flows: [Clock] ─┐
            [Joint State] ├──→ PUB: tcp://*:25556 ───┐
            [Twist] ─┘                              │ Single Address Pair
            [Images] ─────────────────────────────┘ All topics in one data stream
```

#### Data Transmission Format

All data uses the same **multipart message format** with topic-based routing:

```
[Frame 0: topic name (UTF-8)]
[Frame 1: serialized data (JSON or metadata)]
[Frame 2: (optional) raw binary data for images]
```

Example message contents:

- Topic: `clock` → Frame 1: `{"sec": 123, "nanosec": 456000000}`
- Topic: `joint_states` → Frame 1: `{"name": [...], "position": [...], ...}`
- Topic: `cmd_vel` → Frame 1: `{"linear": [...], "angular": [...]}`
- Topic: `rgb_image` → Frame 1: `{metadata}`, Frame 2: `[image bytes]`

#### Socket Pooling

Sockets are cached globally by address. This means:

- Multiple nodes publishing to the same global address reuse the same socket
- No duplicate connections or bind conflicts
- Efficient resource usage

#### Graceful Shutdown

The ZmqManager automatically:

1. Registers cleanup with `atexit` handler
2. Sets socket linger time to prevent data loss
3. Closes all sockets and terminates context on shutdown

### Subscription Protocol (Control Topic)

This section documents the full subscription protocol used by the bridge.

#### Overview

This protocol lets Isaac Sim request that a ROS2 bridge subscribe to specific ROS2 topics and forward data back over ZMQ.

**Control Topic**: `__zmq_bridge_control__`

#### Architecture

```
┌──────────────────┐                    ┌──────────────────┐
│   Isaac Sim      │                    │   ROS2 Bridge    │
│   ZMQ Bridge     │                    │   Node           │
└──────────────────┘                    └──────────────────┘
		 │                                       │
		 │  1. Subscribe Request                │
		 │  topic: __zmq_bridge_control__       │
		 │  {action: "subscribe",               │
		 │   topic: "cmd_vel",                  │
		 │   msg_type: "Twist"}                 │
		 ├──────────────────────────────────────>│
		 │                                       │
		 │                               2. ROS2 Bridge:
		 │                               - Subscribes to /cmd_vel
		 │                               - Forwards to ZMQ
		 │                                       │
		 │  3. Data Flow                         │
		 │  topic: cmd_vel                       │
		 │  {linear: [...], angular: [...]}     │
		 │<──────────────────────────────────────┤
		 │                                       │
```

#### Control Message Format

Subscription request:

```json
{
  "action": "subscribe",
  "topic": "cmd_vel",
  "msg_type": "Twist",
  "timestamp": 1708418400.123
}
```

Unsubscription request:

```json
{
  "action": "unsubscribe",
  "topic": "cmd_vel",
  "timestamp": 1708418400.456
}
```

Fields:

- `action` (string): "subscribe" or "unsubscribe"
- `topic` (string): ROS2 topic name (e.g., "cmd_vel", "joint_states")
- `msg_type` (string): ROS2 message type hint (e.g., "Twist", "JointState", "generic")
- `timestamp` (float): Unix timestamp

#### Isaac Sim Side (Automatic)

ZmqSubscribe nodes send one subscription request on first execution.

```python
# In ZmqSubscribeJointState.compute()
if not state.subscription_requested:
	state.mgr.request_subscription(topic, "JointState")
	state.subscription_requested = True
```

#### ROS2 Bridge Side (Implementation)

The ROS2 bridge must:

1. Subscribe to `__zmq_bridge_control__`
2. Parse control messages
3. Create or destroy ROS2 subscriptions
4. Forward ROS2 messages to ZMQ topics

Minimal Python example:

```python
import zmq
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState


class ZmqRos2Bridge(Node):
	def __init__(self):
		super().__init__("zmq_ros2_bridge")

		ctx = zmq.Context()
		self.zmq_pub = ctx.socket(zmq.PUB)
		self.zmq_pub.bind("tcp://*:25557")

		self.zmq_sub = ctx.socket(zmq.SUB)
		self.zmq_sub.connect("tcp://isaac-sim:25556")
		self.zmq_sub.setsockopt(zmq.SUBSCRIBE, b"__zmq_bridge_control__")

		self.ros2_subscriptions = {}
		self.create_timer(0.01, self.check_control_messages)

	def check_control_messages(self):
		if self.zmq_sub.poll(0):
			topic_b, data_b = self.zmq_sub.recv_multipart()
			topic = topic_b.decode("utf-8")

			if topic == "__zmq_bridge_control__":
				msg = json.loads(data_b.decode("utf-8"))
				self.handle_control_message(msg)

	def handle_control_message(self, msg):
		action = msg.get("action")
		topic = msg.get("topic")
		msg_type = msg.get("msg_type", "generic")

		if action == "subscribe":
			self.get_logger().info(f"Subscription request: {topic} ({msg_type})")
			self.create_ros2_subscription(topic, msg_type)
		elif action == "unsubscribe":
			self.get_logger().info(f"Unsubscription request: {topic}")
			self.remove_ros2_subscription(topic)

	def create_ros2_subscription(self, topic, msg_type):
		if topic in self.ros2_subscriptions:
			return

		msg_class = self.get_msg_class(msg_type)

		def callback(msg):
			data = self.ros2_to_dict(msg, msg_type)
			self.zmq_pub.send_multipart([
				topic.encode("utf-8"),
				json.dumps(data).encode("utf-8"),
			])

		sub = self.create_subscription(msg_class, topic, callback, 10)
		self.ros2_subscriptions[topic] = sub
		self.get_logger().info(f"Created ROS2 subscription: {topic}")

	def remove_ros2_subscription(self, topic):
		if topic in self.ros2_subscriptions:
			self.destroy_subscription(self.ros2_subscriptions[topic])
			del self.ros2_subscriptions[topic]
			self.get_logger().info(f"Removed ROS2 subscription: {topic}")

	def get_msg_class(self, msg_type):
		type_map = {
			"Twist": Twist,
			"JointState": JointState,
		}
		return type_map.get(msg_type, Twist)

	def ros2_to_dict(self, msg, msg_type):
		if msg_type == "Twist":
			return {
				"linear": [msg.linear.x, msg.linear.y, msg.linear.z],
				"angular": [msg.angular.x, msg.angular.y, msg.angular.z],
			}
		if msg_type == "JointState":
			return {
				"name": list(msg.name),
				"position": list(msg.position),
				"velocity": list(msg.velocity),
			}
		return {}


def main():
	rclpy.init()
	bridge = ZmqRos2Bridge()
	rclpy.spin(bridge)
	bridge.destroy_node()
	rclpy.shutdown()


if __name__ == "__main__":
	main()
```

#### Protocol Flow

1. Isaac Sim starts and initializes global addresses.
2. ZmqSubscribe node runs and sends a control message to request a ROS2 subscription.
3. ROS2 bridge creates a ROS2 subscriber and forwards messages to ZMQ.
4. Isaac Sim receives ZMQ data on the requested topic.

#### Message Type Mapping

| Isaac Sim Node         | msg_type Hint | ROS2 Message Type      |
| ---------------------- | ------------- | ---------------------- |
| ZmqSubscribeJointState | "JointState"  | sensor_msgs/JointState |
| ZmqSubscribeTwist      | "Twist"       | geometry_msgs/Twist    |
| ZmqSubscribeFloat      | "Float32"     | std_msgs/Float32       |
| ZmqSubscribeDouble     | "Float64"     | std_msgs/Float64       |
| ZmqSubscribeInt        | "Int32"       | std_msgs/Int32         |
| Custom nodes           | "generic"     | Implementation-defined |

#### Troubleshooting

- Verify control topic messages are received on `__zmq_bridge_control__`.
- Confirm ZMQ addresses match between Isaac Sim and ROS2 bridge.
- Check ROS2 logs for subscription creation events.
- Ensure ROS2 topic is publishing data.

### Transport

- ZMQ PUB/SUB
- Message format: multipart frames
  - Frame 0: topic (UTF-8 string)
  - Frame 1: payload (JSON UTF-8)
  - For image: Frame 1 = metadata JSON, Frame 2 = raw bytes

### Common Fields

- `topicName`: ZMQ topic string
- `pubAddress`: ZMQ PUB address, example: tcp://*:25556
- `subAddress`: ZMQ SUB address, example: tcp://127.0.0.1:25557

### Topics and Payloads

#### Joint State

- Node: ZmqPublishJointState / ZmqSubscribeJointState
- Topic default: `joint_states`
- Payload JSON:
  - `name`: array of string
  - `position`: array of float
  - `velocity`: array of float

Example:

```json
{
	"name": ["joint1", "joint2"],
	"position": [0.1, 0.2],
	"velocity": [0.0, 0.0]
}
```

#### Twist

- Node: ZmqPublishTwist / ZmqSubscribeTwist
- Topic default: `cmd_vel`
- Payload JSON:
  - `linear`: array of float[3]
  - `angular`: array of float[3]

Example:

```json
{
	"linear": [0.5, 0.0, 0.0],
	"angular": [0.0, 0.0, 0.2]
}
```

#### Clock

- Node: ZmqPublishClock
- Topic default: `clock`
- Payload JSON:
  - `sec`: integer
  - `nanosec`: integer

Example:

```json
{
	"sec": 12,
	"nanosec": 345000000
}
```

#### Camera Image (RGBA)

- Node: ZmqCameraHelper
- Topic default: `rgb_image`
- Multipart 帧:
  - Frame 0: topic (e.g. `rgb_image`)
  - Frame 1: metadata JSON
  - Frame 2: PNG encoded image bytes (includes Alpha channel, BGRA before encoding)

Metadata JSON fields:

- `width`: integer
- `height`: integer
- `channels`: integer (4 for rgba)
- `encoding`: string, fixed: `png`
- `frame_id`: string
- `type`: string, fixed `image`

Example metadata:

```json
{
	"width": 1280,
	"height": 720,
	"channels": 4,
	"encoding": "png",
	"frame_id": "sim_camera",
	"type": "image"
}
```

#### IMU

- Node: ZmqImuHelper
- Topic default: `imu_data`
- Inputs:
  - `imuPrim`: Target IMU sensor prim
  - `frameId`: Header frame ID
- Payload JSON:
  - `header`: Object containing `frame_id` and `stamp`
  - `orientation`: Object with x, y, z, w quaternion
  - `angular_velocity`: Object with x, y, z components
  - `linear_acceleration`: Object with x, y, z components
  - `type`: string, fixed `imu`

Example:

```json
{
  "header": { "frame_id": "imu_frame", "stamp": 0.0 },
  "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 },
  "angular_velocity": { "x": 0.0, "y": 0.0, "z": 0.0 },
  "linear_acceleration": { "x": 0.0, "y": 0.0, "z": 9.81 },
  "type": "imu"
}
```

#### Primitive Types (Float, Double, Int)

The bridge supports publishing and subscribing to basic primitive types.

**Nodes**:

- `ZmqPublishFloat` / `ZmqSubscribeFloat` (msg_type hint: "Float32")
- `ZmqPublishDouble` / `ZmqSubscribeDouble` (msg_type hint: "Float64")
- `ZmqPublishInt` / `ZmqSubscribeInt` (msg_type hint: "Int32")

**Payload JSON**:

```json
{
  "data": 123.456
}
```

**Common Inputs**:

- `data`: The value to publish (float, double, or int).

**Common Outputs**:

- `data`: The received value.

#### Primitive Array Types (Float[], Double[], Int[])

The bridge supports publishing and subscribing to arrays of primitive types.

**Nodes**:

- `ZmqPublishFloatArray` / `ZmqSubscribeFloatArray` (msg_type hint: "Float32MultiArray")
- `ZmqPublishDoubleArray` / `ZmqSubscribeDoubleArray` (msg_type hint: "Float64MultiArray")
- `ZmqPublishIntArray` / `ZmqSubscribeIntArray` (msg_type hint: "Int32MultiArray")

**Payload JSON**:

```json
{
  "data": [1.1, 2.2, 3.3]
}
```

**Common Inputs**:

- `data`: The array to publish (float[], double[], or int[]).

**Common Outputs**:

- `data`: The received array.

#### Nav/GPS

- Node: ZmqNavHelper
- Default Topic: `gps_fix`
- Payload JSON:
  - `header`: Object containing `frame_id` and `stamp`
  - `status`: Object containing `status` (0) and `service` (1)
  - `latitude`: double
  - `longitude`: double
  - `altitude`: double
  - `position_covariance`: double[9]
  - `position_covariance_type`: int (0)
  - `type`: string, fixed `gps`

**Inputs**:

- `targetPrim`: The prim to track (e.g. vehicle base_link).
- `originLatitude`: Latitude of the world origin (0,0,0) in degrees.
- `originLongitude`: Longitude of the world origin (0,0,0) in degrees.
- `originAltitude`: Altitude of the world origin (0,0,0) in meters.

Example:

```json
{
  "header": { "frame_id": "gps_frame", "stamp": 0.0 },
  "status": { "status": 0, "service": 1 },
  "latitude": 37.7749,
  "longitude": -122.4194,
  "altitude": 10.5,
  "position_covariance": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  "position_covariance_type": 0,
  "type": "gps"
}
```

#### Depth & Altimeter

- Nodes: `ZmqDepthHelper` (for depth), `ZmqAltimeterHelper` (for altitude)
- Default Topics: `depth` (Depth), `altitude` (Altimeter)
- Payload JSON:
  - `data`: float (meters)

**Logic**:

- **Depth**: `waterSurfaceZ - currentZ`. Returns 0.0 if currentZ > waterSurfaceZ (above water).
- **Altimeter**: `currentZ - waterSurfaceZ`. Returns 0.0 if currentZ < waterSurfaceZ (below water).

**Inputs**:

- `targetPrim`: The prim to measure.
- `waterSurfaceZ`: The Z coordinate of the water surface (default 0.0).

Example:

```json
{
  "data": 15.4
}
```

#### Odometry

- Node: `ZmqOdometryHelper`
- Default Topic: `odom`
- Input: `targetPrim` (Rigid Body Prim)
- Payload JSON:
  - `header`: frame_id, stamp
  - `child_frame_id`: string
  - `pose`: Object containing `pose` (position, orientation) and `covariance`
  - `twist`: Object containing `twist` (linear, angular) and `covariance`

Example:

```json
{
  "header": { "frame_id": "odom", "stamp": 0.0 },
  "child_frame_id": "base_link",
  "pose": {
    "pose": {
      "position": { "x": 1.0, "y": 2.0, "z": 0.0 },
      "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }
    },
    "covariance": [0.0, ...]
  },
  "twist": {
    "twist": {
      "linear": { "x": 0.5, "y": 0.0, "z": 0.0 },
      "angular": { "x": 0.0, "y": 0.0, "z": 0.1 }
    },
    "covariance": [0.0, ...]
  },
  "type": "odom"
}
```

### ROS2 Python Examples

以下为最小化 ROS2 Python 示例，演示 ZMQ 与 ROS2 之间的桥接。安装依赖：

```bash
pip install pyzmq
```

#### ZMQ -> ROS2 (Twist)

订阅 ZMQ `cmd_vel`，发布 ROS2 `geometry_msgs/Twist`。

```python
import json
import zmq
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ZmqToRos2Twist(Node):
	def __init__(self, sub_address="tcp://127.0.0.1:25557", topic="cmd_vel"):
		super().__init__("zmq_to_ros2_twist")
		self.pub = self.create_publisher(Twist, "cmd_vel", 10)

		ctx = zmq.Context.instance()
		self.sub = ctx.socket(zmq.SUB)
		self.sub.connect(sub_address)
		self.sub.setsockopt(zmq.SUBSCRIBE, topic.encode("utf-8"))

		self.timer = self.create_timer(0.01, self._tick)

	def _tick(self):
		try:
			if self.sub.poll(0):
				topic_b, payload_b = self.sub.recv_multipart()
				data = json.loads(payload_b.decode("utf-8"))

				msg = Twist()
				msg.linear.x, msg.linear.y, msg.linear.z = data.get("linear", [0.0, 0.0, 0.0])
				msg.angular.x, msg.angular.y, msg.angular.z = data.get("angular", [0.0, 0.0, 0.0])
				self.pub.publish(msg)
		except Exception as exc:
			self.get_logger().warn(f"ZMQ receive failed: {exc}")


def main():
	rclpy.init()
	node = ZmqToRos2Twist()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()


if __name__ == "__main__":
	main()
```

#### ROS2 -> ZMQ (Twist)

订阅 ROS2 `cmd_vel`，发布 ZMQ `cmd_vel`。

```python
import json
import zmq
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class Ros2ToZmqTwist(Node):
	def __init__(self, pub_address="tcp://*:25556", topic="cmd_vel"):
		super().__init__("ros2_to_zmq_twist")
		self.topic = topic
		self.sub = self.create_subscription(Twist, "cmd_vel", self._on_twist, 10)

		ctx = zmq.Context.instance()
		self.pub = ctx.socket(zmq.PUB)
		self.pub.bind(pub_address)

	def _on_twist(self, msg: Twist):
		payload = {
			"linear": [msg.linear.x, msg.linear.y, msg.linear.z],
			"angular": [msg.angular.x, msg.angular.y, msg.angular.z],
		}
		self.pub.send_multipart([
			self.topic.encode("utf-8"),
			json.dumps(payload).encode("utf-8"),
		])


def main():
	rclpy.init()
	node = Ros2ToZmqTwist()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()


if __name__ == "__main__":
	main()
```

## 中文

### ZMQ 上下文管理 - 统一地址模式

ZeroMQ Bridge 使用一个集中的 **ZmqManager** 单例进行生命周期和资源管理。所有数据通过**单一地址对** (PUB/SUB) 传输，通过 **topic name** 区分不同数据类型。

#### 统一地址模式

所有数据流通过**单一地址对**：

- **发布者 (PUB)**：`tcp://*:25556` - 所有发布数据的单一地址
- **订阅者 (SUB)**：`tcp://127.0.0.1:25557` - 所有订阅数据的单一地址

不同数据类型通过消息中的 **topic name** 区分。

**优势**：

- ✅ 单一配置点（ZmqContext）
- ✅ 易于扩展：添加新数据类型无需地址管理
- ✅ 减少端口占用
- ✅ ROS2 桥接配置更简单
- ✅ 自动 socket 池化防止重复

#### 节点：ZmqContext

**ZmqContext** 节点必须放在 Action Graph 的开始，以初始化全局 ZMQ 地址。

**输入**：

- `execIn` (execution)：执行触发输入
- `pubAddress` (string)：全局 ZMQ PUB 绑定地址。所有发布节点将使用此地址。默认：`tcp://*:25556`
- `subAddress` (string)：全局 ZMQ SUB 连接地址。所有订阅节点将使用此地址。默认：`tcp://127.0.0.1:25557`
- `lingerMs` (int)：Socket linger 时间用于优雅关闭。默认：`100ms`

**输出**：

- `execOut` (execution)：执行输出，用于链接到其他节点
- `outPubAddress` (string)：当前全局发布者地址
- `outSubAddress` (string)：当前全局订阅者地址

#### 全局地址管理

ZmqContext 执行后：

- 所有后续**发布节点**自动使用全局 PUB 地址
- 所有后续**订阅节点**自动使用全局 SUB 地址
- 节点仍可选择覆盖全局地址（高级用法）

```
ZmqContext 设置: global_pub = "tcp://*:25556", global_sub = "tcp://127.0.0.1:25557"
                 ↓
所有其他节点自动使用这些地址（socket 池化防止重复）
                 ↓
数据流: [Clock] ─┐
        [JointState] ├──→ PUB: tcp://*:25556 ───┐
        [Twist] ─┘                              │ 单一地址对
        [Images] ─────────────────────────────┘ 所有 topics
```

#### Socket 池化

Socket 按地址全局缓存。这意味着：

- 多个节点发布到相同的全局地址时复用同一个 socket
- 无重复连接或绑定冲突
- 高效的资源使用

#### 优雅关闭

ZmqManager 自动执行：

1. 用 `atexit` 处理器注册清理
2. 设置 Socket linger 时间以防止数据丢失
3. 在关闭时关闭所有 Socket 并终止上下文

### 传输方式

- ZMQ PUB/SUB
- 消息格式：multipart 帧
  - 帧 0：topic（UTF-8 字符串）
  - 帧 1：负载（JSON UTF-8）
  - 图像：帧 1 为 metadata JSON，帧 2 为原始字节

### 通用字段

- `topicName`：ZMQ 话题字符串
- `pubAddress`：ZMQ PUB 地址，例如 tcp://*:25556
- `subAddress`：ZMQ SUB 地址，例如 tcp://127.0.0.1:25557

### 话题与负载

#### Joint State

- 节点：ZmqPublishJointState / ZmqSubscribeJointState
- 默认话题：`joint_states`
- 负载 JSON：
  - `name`：字符串数组
  - `position`：浮点数组
  - `velocity`：浮点数组

示例：

```json
{
	"name": ["joint1", "joint2"],
	"position": [0.1, 0.2],
	"velocity": [0.0, 0.0]
}
```

#### Twist

- 节点：ZmqPublishTwist / ZmqSubscribeTwist
- 默认话题：`cmd_vel`
- 负载 JSON：
  - `linear`: float[3] 数组
  - `angular`: float[3] 数组

示例：

```json
{
	"linear": [0.5, 0.0, 0.0],
	"angular": [0.0, 0.0, 0.2]
}
```

#### Clock

- 节点：ZmqPublishClock
- 默认话题：`clock`
- 负载 JSON：
  - `sec`: 整数
  - `nanosec`: 整数

示例：

```json
{
	"sec": 12,
	"nanosec": 345000000
}
```

#### Camera Image (RGBA)

- 节点：ZmqCameraHelper
- 默认话题：`rgb_image`
- Multipart 帧：
  - 帧 0：topic (例如 `rgb_image`)
  - 帧 1：metadata JSON
  - 帧 2：PNG 编码后的图像字节（包含 Alpha 通道，编码前为 BGRA 格式）

Metadata JSON 字段：

- `width`：整数
- `height`：整数
- `channels`：整数（RGBA 为 4）
- `encoding`：字符串，固定为 `png`
- `frame_id`：字符串
- `type`: 字符串，固定 `image`

示例 metadata：

```json
{
	"width": 1280,
	"height": 720,
	"channels": 4,
	"encoding": "png",
	"frame_id": "sim_camera",
	"type": "image"
}
```

#### 基础类型 (Float, Double, Int)

桥接器支持发布和订阅基本的基础类型数据。

**节点**：

- `ZmqPublishFloat` / `ZmqSubscribeFloat` (msg_type 提示: "Float32")
- `ZmqPublishDouble` / `ZmqSubscribeDouble` (msg_type 提示: "Float64")
- `ZmqPublishInt` / `ZmqSubscribeInt` (msg_type 提示: "Int32")

**负载 JSON**:

```json
{
  "data": 123.456
}
```

**通用输入**:

- `data`: 要发布的数据 (float, double, 或 int)。

**通用输出**:

- `data`: 接收到的数据。

#### 基础数组类型 (Float[], Double[], Int[])

桥接器支持发布和订阅基础类型的数组数据。

**节点**：

- `ZmqPublishFloatArray` / `ZmqSubscribeFloatArray` (msg_type 提示: "Float32MultiArray")
- `ZmqPublishDoubleArray` / `ZmqSubscribeDoubleArray` (msg_type 提示: "Float64MultiArray")
- `ZmqPublishIntArray` / `ZmqSubscribeIntArray` (msg_type 提示: "Int32MultiArray")

**负载 JSON**:

```json
{
  "data": [1.1, 2.2, 3.3]
}
```

**通用输入**:

- `data`: 要发布的数组 (float[], double[], 或 int[])。

**通用输出**:

- `data`: 接收到的数组。

#### 导航/GPS (Nav/GPS)

- 节点：ZmqNavHelper
- 默认话题：`gps_fix`
- 负载 JSON：
  - `header`：包含 `frame_id` 和 `stamp` 的对象
  - `status`：包含 `status` (0) 和 `service` (1) 的对象
  - `latitude`：双精度浮点数 (经度)
  - `longitude`：双精度浮点数 (纬度)
  - `altitude`：双精度浮点数 (高度)
  - `position_covariance`：双精度浮点数组 (位置协方差)
  - `position_covariance_type`：整数 (位置协方差类型)
  - `type`：字符串，固定为 `gps`

**输入**:

- `targetPrim`：需要追踪的目标 Prim（例如车辆的 base_link）。
- `originLatitude`：世界原点 (0,0,0) 的纬度（度）。
- `originLongitude`：世界原点 (0,0,0) 的经度（度）。
- `originAltitude`：世界原点 (0,0,0) 的高度（米）。

示例：

```json
{
  "header": { "frame_id": "gps_frame", "stamp": 0.0 },
  "status": { "status": 0, "service": 1 },
  "latitude": 37.7749,
  "longitude": -122.4194,
  "altitude": 10.5,
  "position_covariance": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  "position_covariance_type": 0,
  "type": "gps"
}
```

#### 深度计与高度计 (Depth & Altimeter)

- 节点：`ZmqDepthHelper` (深度计), `ZmqAltimeterHelper` (高度计)
- 默认话题：`depth` (深度计), `altitude` (高度计)
- 负载 JSON：
  - `data`：浮点数 (米)

**逻辑**:

- **深度计 (Depth)**: `水面高度 - 当前高度`. 若在水面上方 (`currentZ > waterSurfaceZ`) 则输出 0.0。
- **高度计 (Altimeter)**: `当前高度 - 水面高度`. 若在水面下方 (`currentZ < waterSurfaceZ`) 则输出 0.0。

**输入**:

- `targetPrim`：需要测量的目标 Prim。
- `waterSurfaceZ`：水面的 Z 轴高度 (默认为 0.0)。

示例：

```json
{
  "data": 15.4
}
```

#### 里程计 (Odometry)

- 节点：`ZmqOdometryHelper`
- 默认话题：`odom`
- 输入：`targetPrim` (刚体 Prim)
- 负载 JSON：
  - `header`: frame_id, stamp
  - `child_frame_id`: string
  - `pose`: 包含 `pose` (位置, 姿态) 和 `covariance`
  - `twist`: 包含 `twist` (线速度, 角速度) 和 `covariance`

示例：

```json
{
  "header": { "frame_id": "odom", "stamp": 0.0 },
  "child_frame_id": "base_link",
  "pose": {
    "pose": {
      "position": { "x": 1.0, "y": 2.0, "z": 0.0 },
      "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }
    },
    "covariance": [0.0, ...]
  },
  "twist": {
    "twist": {
      "linear": { "x": 0.5, "y": 0.0, "z": 0.0 },
      "angular": { "x": 0.0, "y": 0.0, "z": 0.1 }
    },
    "covariance": [0.0, ...]
  },
  "type": "odom"
}
```

### ROS2 Python 样例

以下为最小化 ROS2 Python 示例，演示 ZMQ 与 ROS2 之间的桥接。安装依赖：

```bash
pip install pyzmq
```

#### ZMQ -> ROS2 (Twist)

订阅 ZMQ `cmd_vel`，发布 ROS2 `geometry_msgs/Twist`。

```python
import json
import zmq
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ZmqToRos2Twist(Node):
	def __init__(self, sub_address="tcp://127.0.0.1:25557", topic="cmd_vel"):
		super().__init__("zmq_to_ros2_twist")
		self.pub = self.create_publisher(Twist, "cmd_vel", 10)

		ctx = zmq.Context.instance()
		self.sub = ctx.socket(zmq.SUB)
		self.sub.connect(sub_address)
		self.sub.setsockopt(zmq.SUBSCRIBE, topic.encode("utf-8"))

		self.timer = self.create_timer(0.01, self._tick)

	def _tick(self):
		try:
			if self.sub.poll(0):
				topic_b, payload_b = self.sub.recv_multipart()
				data = json.loads(payload_b.decode("utf-8"))

				msg = Twist()
				msg.linear.x, msg.linear.y, msg.linear.z = data.get("linear", [0.0, 0.0, 0.0])
				msg.angular.x, msg.angular.y, msg.angular.z = data.get("angular", [0.0, 0.0, 0.0])
				self.pub.publish(msg)
		except Exception as exc:
			self.get_logger().warn(f"ZMQ receive failed: {exc}")


def main():
	rclpy.init()
	node = ZmqToRos2Twist()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()


if __name__ == "__main__":
	main()
```

#### ROS2 -> ZMQ (Twist)

订阅 ROS2 `cmd_vel`，发布 ZMQ `cmd_vel`。

```python
import json
import zmq
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class Ros2ToZmqTwist(Node):
	def __init__(self, pub_address="tcp://*:25556", topic="cmd_vel"):
		super().__init__("ros2_to_zmq_twist")
		self.topic = topic
		self.sub = self.create_subscription(Twist, "cmd_vel", self._on_twist, 10)

		ctx = zmq.Context.instance()
		self.pub = ctx.socket(zmq.PUB)
		self.pub.bind(pub_address)

	def _on_twist(self, msg: Twist):
		payload = {
			"linear": [msg.linear.x, msg.linear.y, msg.linear.z],
			"angular": [msg.angular.x, msg.angular.y, msg.angular.z],
		}
		self.pub.send_multipart([
			self.topic.encode("utf-8"),
			json.dumps(payload).encode("utf-8"),
		])


def main():
	rclpy.init()
	node = Ros2ToZmqTwist()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()


if __name__ == "__main__":
	main()
```
