---
title: "The ROS 2 Communication Graph: Nodes, Topics, & Services"
sidebar_position: 2
tags: [ROS 2, Robotics, Middleware, Nodes, Topics, Services, QoS, rclpy]
---

# Chapter 1: The ROS 2 Communication Graph

Welcome to the architectural core of the Robot Operating System 2 (ROS 2). A modern robot is a deeply complex system with numerous sensors, actuators, and algorithms all running in parallel. ROS 2 provides the essential framework—the "nervous system"—that allows these disparate parts to communicate and work together as a cohesive whole.

At the heart of this framework is the **ROS 2 Graph**, a conceptual network of all the running processes and the communication channels that connect them. To build any robotic application, you must first master the three fundamental elements that constitute this graph: **Nodes**, **Topics**, and **Services**.

## 1.1 ROS 2 Nodes: The Units of Computation

A **Node** is the principal building block of a ROS 2 system. It is an independent, executable process responsible for a single, well-defined task. By breaking a complex system into many small, modular nodes, we achieve several key advantages:
- **Reusability:** A node for controlling a specific camera can be reused across different robots.
- **Fault Isolation:** If a single node crashes (e.g., the path planning node), the rest of the system (like motor control and sensor drivers) can continue to operate.
- **Clarity and Maintainability:** It is far easier to understand, debug, and maintain a system composed of simple, single-purpose workers than a single monolithic program.

**Analogy:** Think of a node as a specialized department in a company. You might have the "Perception Department" (a node processing camera data), the "Mobility Department" (a node controlling the wheels), and the "Planning Department" (a node that decides where to go). Each has its own job, and they communicate to achieve a collective goal.

### Creating a Minimal Node with `rclpy`

The `rclpy` library is the official Python client library for interacting with the ROS 2 graph. Let's examine the structure of a minimal node.

```python
# file: minimal_node.py
import rclpy
from rclpy.node import Node

# All nodes are classes that inherit from rclpy.node.Node
class MyMinimalNode(Node):
    def __init__(self):
        # 1. Initialize the parent Node class and give the node a unique name
        super().__init__('minimal_node')
        # 2. Use the node's built-in logger to print a message
        self.get_logger().info('My minimal node is now running.')

def main(args=None):
    # 3. Initialize the rclpy library
    rclpy.init(args=args)
    # 4. Create an instance of our node
    node = MyMinimalNode()
    
    try:
        # 5. "Spin" the node, keeping it alive to handle communications
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 6. Cleanly destroy the node and shut down rclpy
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```
The `rclpy.spin(node)` function is the engine of a ROS 2 node. It enters a loop, processing any incoming messages or service calls and ensuring the node remains active.

---

## 1.2 Topics: The Public Data Stream (Asynchronous, Many-to-Many)

**Topics** are the most common communication method in ROS 2. They are named channels, or "buses," over which nodes can exchange data in a decoupled manner.

- **Publishers** are nodes that send messages to a topic.
- **Subscribers** are nodes that receive messages from a topic.

A single topic can have many publishers and many subscribers. The nodes do not know about each other's existence; they only know about the topic. This makes the system highly modular.

**Analogy:** A topic is like a public radio station. A radio host (a publisher) broadcasts on a specific frequency (the topic name), like "98.7 FM News." Any number of listeners (subscribers) can tune their radios to that frequency to receive the broadcast. The host doesn't know who is listening, and the listeners don't know who the host is—they only care about the frequency.

### Message Types

Every topic is strongly typed. It can only transmit messages of a specific, predefined structure. These structures are defined in `.msg` files. For example, `std_msgs/msg/String` is a simple message type that contains a single field: `string data`. More complex types like `sensor_msgs/msg/Image` contain fields for image dimensions, encoding, and the image data itself.

### Quality of Service (QoS): Controlling Data Delivery

Real-world networks can be unreliable. What if a sensor message is dropped? For some data, this is acceptable; for others, it's a critical failure. **Quality of Service (QoS)** policies are rules that let you control how ROS 2 handles the transmission of messages.

**Analogy:** Think of QoS as choosing a delivery service.
- **Standard Mail (Best-Effort):** Fast and cheap, but a letter might occasionally get lost.
- **Certified Mail (Reliable):** Slower and more resource-intensive, but delivery is guaranteed.

The two most important QoS policies are:
1.  **Reliability:**
    -   `BEST_EFFORT`: Recommended for high-frequency, non-critical data like video streams or sensor readings, where losing an occasional message has little impact.
    -   `RELIABLE`: Ensures that every message is delivered. This is essential for critical commands (like "stop motor") or important state changes.
2.  **History & Depth:**
    -   `KEEP_LAST`: Only keeps a small number of recent messages for late-joining subscribers. The `depth` parameter specifies how many.
    -   `KEEP_ALL`: Keeps all messages, but can consume significant memory.

You can specify a QoS profile when creating a publisher or subscriber to fine-tune this behavior.

### Example: A QoS-Aware Publisher and Subscriber

**Publisher Node (`chatter_publisher.py`):**
```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String

class ChatterPublisher(Node):
    def __init__(self):
        super().__init__('chatter_publisher')
        
        # Define a QoS profile: reliable, keeping the last 10 messages.
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.publisher_ = self.create_publisher(String, 'chatter', qos_profile)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello from publisher: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

# ... (main function as before)
```

**Subscriber Node (`chatter_subscriber.py`):**
```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String

class ChatterSubscriber(Node):
    def __init__(self):
        super().__init__('chatter_subscriber')
        
        # The subscriber's QoS profile must be compatible with the publisher's.
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            qos_profile)

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

# ... (main function as before)
```

---

## 1.3 Services: The RPC-Style Transaction (Synchronous, One-to-One)

While topics are ideal for continuous data streams, they are unsuitable for synchronous request/reply interactions. For this, ROS 2 provides **Services**. A service allows one node (the **Client**) to send a request to another node (the **Server**) and wait for a response.

**Analogy:** If a topic is a radio broadcast, a service is a direct phone call or a web API request. You (the client) call a specific number (the service name), ask a question (the request), and wait on the line until you get an answer (the response).

### Service Types

Similar to messages, service types are defined in `.srv` files. The structure is split into two parts (request and response), separated by `---`.

**`example_interfaces/srv/AddTwoInts.srv`**
```
# Request
int64 a
int64 b
---
# Response
int64 sum
```

### Example: An "Add Two Integers" Service

**Service Server (`add_server.py`):**
The server waits for requests, processes them in a callback, and fills out the response.
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddIntsServer(Node):
    def __init__(self):
        super().__init__('add_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Service server is ready.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}. Returning sum={response.sum}')
        return response

# ... (main function)
```

**Service Client (`add_client.py`):**
The client sends a request and waits asynchronously for the future result.
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
import sys

class AddIntsClient(Node):
    def __init__(self):
        super().__init__('add_ints_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        # Wait until the service is actually available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        # Call the service asynchronously and wait for the result
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    client_node = AddIntsClient()
    response = client_node.send_request(int(sys.argv[1]), int(sys.argv[2]))
    client_node.get_logger().info(
        f'Result of add_two_ints: for {sys.argv[1]} + {sys.argv[2]} = {response.sum}')
    client_node.destroy_node()
    rclpy.shutdown()
# ... (main function call)
```

> **Warning:** A service callback is blocking. If a calculation takes a long time, the server cannot respond to any other clients. For long-running tasks that require feedback and can be cancelled (e.g., "navigate to a location"), you should use **Actions**, a more advanced communication pattern.

## 1.4 Choosing the Right Communication Pattern

| Pattern  | Style         | Relationship | Use Case                                  | Analogy              |
|----------|---------------|--------------|-------------------------------------------|----------------------|
| **Topic**  | Asynchronous  | Many-to-Many | Continuous streams of data (sensor readings, robot state). Fire-and-forget. | Radio Station |
| **Service**| Synchronous   | One-to-One   | Quick, transactional request/reply interactions (triggering a calculation, getting a simple state). | Phone Call / API Request |
| **Action** | Asynchronous  | One-to-One   | Long-running tasks with feedback and preemption (navigation, manipulation). | Ordering a Pizza (Order -> Cooking -> Delivered) |

## 1.5 Command-Line Inspection Tools

ROS 2 provides powerful command-line tools to debug the graph:
- **Nodes:** `ros2 node list`
- **Topics:** `ros2 topic list -t` (shows types), `ros2 topic echo <topic_name>`
- **Services:** `ros2 service list -t` (shows types), `ros2 service call <service_name> <srv_type> '{json_args}'`
- **Interfaces:** `ros2 interface show <msg/srv_type>` (shows the structure)

By mastering Nodes, Topics, Services, and their associated tools, you have a solid foundation for building any robotics application in ROS 2.