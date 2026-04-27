import zmq
import json
import time
import threading

def subscriber_thread():
    """模拟 Isaac Sim 的数据接收端 (SUB)"""
    ctx = zmq.Context.instance()
    sub = ctx.socket(zmq.SUB)
    # Bridge 绑定在 25557 发布数据，我们连接它
    sub.connect("tcp://127.0.0.1:25557")
    # 订阅所有话题 (或者指定 topic.encode('utf-8'))
    sub.setsockopt(zmq.SUBSCRIBE, b"")
    
    print("[SUB] Listening for data on tcp://127.0.0.1:25557 ...")
    
    while True:
        try:
            # 非阻塞接收，方便退出
            if sub.poll(100):
                parts = sub.recv_multipart()
                topic = parts[0].decode('utf-8')
                payload = parts[1].decode('utf-8')
                print(f"\n[SUB] RECEIVED DATA:\n  Topic: {topic}\n  Payload: {payload}")
        except Exception as e:
            print(f"[SUB] Error: {e}")

# 1. 启动接收线程
sub_thread = threading.Thread(target=subscriber_thread, daemon=True)
sub_thread.start()

# 2. 模拟 Isaac Sim 的控制/发布端 (PUB)
context = zmq.Context.instance()
socket = context.socket(zmq.PUB)
# Isaac Sim 默认绑定 25556 发送控制指令和数据
try:
    socket.bind("tcp://*:25556")
    print("[PUB] Bound to tcp://*:25556 (Control/Data Source)")
except zmq.error.ZMQError:
    print("[PUB] Error: Port 25556 already in use! Is Isaac Sim or another script running?")
    exit(1)

print("Waiting 2s for Bridge to connect...")
time.sleep(2)

# 3. 发送订阅请求 (请求 Bridge 转发 /cmd_vel 给我们)
topic_to_sub = "/cmd_vel"
#去掉前导斜杠以匹配 ZMQ topic 习惯，虽然 Bridge 会处理
zmq_topic = topic_to_sub.lstrip("/") 

cmd = {
    "action": "subscribe", 
    "topic": topic_to_sub, 
    "msg_type": "Twist"
}
print(f"[PUB] Sending Subscription Request: {cmd}")
socket.send_multipart([
    b"__zmq_bridge_control__", 
    json.dumps(cmd).encode("utf-8")
])


# 4. 同时发布数据给 ROS 2 Bridge (例如发布 JointState)
def publisher_thread():
    """模拟 Isaac Sim 的数据发送端 (PUB)"""
    # 复用上面已经绑定的 PUB socket (但 zmq socket 不是线程安全的，不建议跨线程混用)
    # 更好的做法是在主线程发送，或者使用独立的 socket，但这里为了简单复用主线程的上下文
    # 注意：ZMQ PUB 只有一端能 bind，所以我们这里必须复用主线程已经 bind 的 socket
    # 鉴于 python GIL，我们尽量在主循环里发送
    pass

print("\n=== TEST READY ===")
print("1. [ROS2->ZMQ Test] Run: ros2 topic pub -r 1 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0}}'")
print("2. [ZMQ->ROS2 Test] Run: ros2 topic echo /joint_states")
print("3. Verify if this script prints [SUB] RECEIVED DATA...")

print("Running loop for 60 seconds...")

# 模拟发布 JointState 数据 (ZMQ -> ROS2)
joint_msg = {
    "name": ["joint1", "joint2"],
    "position": [0.0, 0.0],
    "velocity": [0.0, 0.0]
}
joint_topic = "joint_states" # 发送给 ROS2 的话题名

start_time = time.time()
while time.time() - start_time < 60:
    # 模拟 JointState 变化
    joint_msg["position"][0] += 0.01
    joint_msg["position"][1] -= 0.01
    
    try:
        # 发送 JointState 数据
        socket.send_multipart([
            joint_topic.encode('utf-8'),
            json.dumps(joint_msg).encode('utf-8')
        ])
        # print(f"[PUB] Sent JointState: {joint_msg['position']}") # 减少刷屏
    except Exception as e:
        print(f"[PUB] Error sending: {e}")
        
    time.sleep(0.1) # 10Hz

print("Test finished.")


