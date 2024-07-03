import sys
import os

# Add the current directory to the Python path
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import pytest
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from py_trees.common import Status
import py_trees_ros as ptr
import py_trees as pt

from behaviours import Rotate, StopMotion2bb, BatteryStatus2bb, LaserScan2bb, create_root
#from state-machine-behavious-and-trees

# Helper class to Main ROS node and publishers
class MainNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.published_messages = {}

    def create_publisher(self, msg_type, topic, qos_profile=QoSProfile(depth=10)):
        def publish(msg):
            if topic not in self.published_messages:
                self.published_messages[topic] = []
            self.published_messages[topic].append(msg)

        publisher = super().create_publisher(msg_type, topic, qos_profile)
        publisher.publish = publish
        return publisher

# Setup pytest fixture to initialize ROS and MainNode
@pytest.fixture(scope="module")
def ros_setup():
    rclpy.init(args=None)
    yield
    rclpy.shutdown()

@pytest.fixture
def Main_node():
    return MainNode("Main_node")

def test_rotate(ros_setup, Main_node):
    rotate = Rotate(name="Rotate", topic_name="/cmd_vel", ang_vel=0.5)
    rotate.setup(node=Main_node)
    
    status = rotate.update()
    assert status == Status.RUNNING

    # Check if a Twist message was published with correct angular velocity
    assert "/cmd_vel" in Main_node.published_messages
    twist_msg = Main_node.published_messages["/cmd_vel"][-1]
    assert twist_msg.angular.z == 0.5

    rotate.terminate(new_status=Status.SUCCESS)
    twist_msg = Main_node.published_messages["/cmd_vel"][-1]
    assert twist_msg.angular.z == 0.0

def test_stop_motion(ros_setup, Main_node):
    stop_motion = StopMotion2bb(name="Stop Motion", topic_name="/cmd_vel")
    stop_motion.setup(node=Main_node)
    
    status = stop_motion.update()
    assert status == Status.SUCCESS

    # Check if a Twist message was published with zero velocity
    assert "/cmd_vel" in Main_node.published_messages
    twist_msg = Main_node.published_messages["/cmd_vel"][-1]
    assert twist_msg.angular.z == 0.0
    assert twist_msg.linear.x == 0.0

def test_battery_status2bb(ros_setup, Main_node):
    battery_status = BatteryStatus2bb(battery_voltage_topic_name="/battery_voltage", threshold=30.0)
    battery_status.setup(node=Main_node)
    
    # Simulate a low battery message
    battery_status.blackboard.battery = 20.0
    status = battery_status.update()
    assert status == Status.FAILURE
    assert battery_status.blackboard.battery_low_warning == True

    # Simulate a sufficient battery message
    battery_status.blackboard.battery = 40.0
    status = battery_status.update()
    assert status == Status.SUCCESS
    assert battery_status.blackboard.battery_low_warning == False

def test_laser_scan2bb(ros_setup, Main_node):
    laser_scan = LaserScan2bb(topic_name="/scan", safe_range=0.25)
    laser_scan.setup(node=Main_node)

    # Simulate a scan message with an obstacle in range
    laser_scan.blackboard.laser_scan = [0.2, 0.3, 0.4]
    status = laser_scan.update()
    assert status == Status.FAILURE

    # Simulate a scan message with no obstacles in range
    laser_scan.blackboard.laser_scan = [0.3, 0.4, 0.5]
    status = laser_scan.update()
    assert status == Status.SUCCESS

def test_create_root(ros_setup, Main_node):
    root = create_root()
    tree = ptr.trees.BehaviourTree(root=root)
    tree.setup(timeout=30.0)
    
    # Ensure the structure of the tree
    assert isinstance(root, pt.composites.Parallel)
    assert len(root.children) == 2
    assert isinstance(root.children[0], pt.composites.Sequence)
    assert isinstance(root.children[1], pt.composites.Selector)

    topics2BB = root.children[0]
    assert len(topics2BB.children) == 2
    assert isinstance(topics2BB.children[0], BatteryStatus2bb)
    assert isinstance(topics2BB.children[1], LaserScan2bb)

    priorities = root.children[1]
    assert len(priorities.children) == 4
    assert isinstance(priorities.children[0], pt.composites.Selector)
    assert isinstance(priorities.children[1], pt.decorators.Condition)
    assert isinstance(priorities.children[2], pt.composites.Sequence)
    assert isinstance(priorities.children[3], pt.behaviours.Running)

    stop_if_needed = priorities.children[0]
    assert len(stop_if_needed.children) == 2
