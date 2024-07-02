### Implement the safety functionalities for the Robile by implementing all
### required behaviours here. Feel free to define additional behaviours if necessary

import os
# Add the current directory to the Python path
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))

import rclpy
import py_trees as pt
import py_trees_ros as ptr
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import ipytest
import pytest
import py_trees



class Rotate(pt.behaviour.Behaviour):
    """Rotates the robot about the z-axis 
    """
    def __init__(self, name="rotate platform",
                 topic_name="/cmd_vel",
                 ang_vel=1.0):
        super(Rotate, self).__init__(name)

        # TODO: initialise any necessary class variables
        # YOUR CODE HERE
#         raise NotImplementedError()
        self.topic_name = topic_name
        self.ang_vel = ang_vel
        self.publisher = None


    def setup(self, **kwargs):
        """Setting up things which generally might require time to prevent delay in the tree initialisation
        """
        self.logger.info("[ROTATE] setting up rotate behaviour")
        try:
            self.node = kwargs['node']
            self.publisher = self.node.create_publisher(Twist, self.topic_name, 10)
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e 
        return True

    def update(self):
        """Rotates the robot at the maximum allowed angular velocity.
        Note: The actual behaviour is implemented here.

        """
        self.logger.info("[ROTATE] update: updating rotate behaviour")
        self.logger.debug("%s.update()" % self.__class__.__name__)

        # TODO: implement the primary function of the behaviour and decide which status to return 
        # based on the structure of your behaviour tree

        # Hint: to return a status, for example, SUCCESS, pt.common.Status.SUCCESS can be used

        # YOUR CODE HERE
        twist = Twist()
        twist.angular.z = self.ang_vel
        self.publisher.publish(twist)
        return pt.common.Status.RUNNING 


    def terminate(self, new_status):
        """Trigerred once the execution of the behaviour finishes, 
        i.e. when the status changes from RUNNING to SUCCESS or FAILURE
        """
        self.logger.info("[ROTATE] terminate: publishing zero angular velocity")

        # TODO: implement the termination of the behaviour, i.e. what should happen when the behaviour 
        # finishes its execution

        twist = Twist()
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        super().terminate(new_status)

class StopMotion2bb(pt.behaviour.Behaviour):
    """Stops the robot when it is controlled using a joystick or with a cmd_vel command
    """
    
    # TODO: Implement a behaviour to stop the robot's motion

    # YOUR CODE HERE
    def __init__(self, name="Stop Motion", topic_name="/cmd_vel"):
        super(StopMotion2bb, self).__init__(name)
        self.topic_name = topic_name
        self.publisher = None
        self.initialized = False

    def update(self):
        if not self.initialized:
            try:
                self.publisher = self.node.create_publisher(Twist, self.topic_name, 10)
                self.initialized = True
            except AttributeError as e:
                self.logger.error(error_message)
                raise AttributeError(error_message) from e

        twist = Twist()
        self.publisher.publish(twist)
        return pt.common.Status.SUCCESS


class BatteryStatus2bb(ptr.subscribers.ToBlackboard):
    """Checks the battery status"""
    def __init__(self, battery_voltage_topic_name: str="/battery_voltage",
                 name: str='Battery2BB',
                 threshold: float=30.0):
        super().__init__(name=name,
                         topic_name=battery_voltage_topic_name,
                         topic_type=Float32,
                         blackboard_variables={'battery': 'data'},
                         initialise_variables={'battery': 100.0},
                         clearing_policy=pt.common.ClearingPolicy.NEVER,
                         qos_profile=ptr.utilities.qos_profile_unlatched())
        self.blackboard.register_key(key='battery_low_warning', access=pt.common.Access.WRITE)
        self.threshold = threshold

    def update(self):
        """Calls the parent to write the raw data to the blackboard and then check against the
        threshold to determine if a low warning flag should also be updated.
        """
        self.logger.info('[BATTERY] update: running battery_status2bb update')
        super().update()
        
        voltage = self.blackboard.battery
        if voltage < self.threshold:
            self.blackboard.battery_low_warning = True
            return pt.common.Status.FAILURE
        else:
            self.blackboard.battery_low_warning = False
            return pt.common.Status.SUCCESS



class LaserScan2bb(ptr.subscribers.ToBlackboard):
    """Checks the laser scan measurements to avoid possible collisions.
    """
    def __init__(self, topic_name: str="/scan",
                 name: str='Scan2BB',
                 safe_range: float=0.25):
        super().__init__(name=name,
                         topic_name=topic_name,
                         topic_type=LaserScan,
                         blackboard_variables={'laser_scan':'ranges'},
                         clearing_policy=pt.common.ClearingPolicy.NEVER,  # to decide when data should be cleared/reset.
                         qos_profile=QoSProfile(reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                                                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                                depth=10))
        
        # TODO: initialise class variables and blackboard variables
        self.safe_range = safe_range

    def update(self):
        # TODO: impletment the update function to check the laser scan data and update the blackboard variable
        super().update()
        ranges = self.blackboard.laser_scan
        if any(r < self.safe_range for r in ranges):
            return pt.common.Status.FAILURE  # Indicates a collision risk
        return pt.common.Status.SUCCESS
    


### Implement a behaviour tree using your previously implemented behaviours here

import py_trees as pt
import py_trees_ros as ptr
import operator

import py_trees.console as console
import rclpy
import sys

def create_root() -> pt.behaviour.Behaviour:
    """Structures a behaviour tree to monitor the battery status, and start
    to rotate if the battery is low and stop if it detects an obstacle in front of it.
    """

    # we define the root node
    root = pt.composites.Parallel(name="root",
                                  policy=pt.common.ParallelPolicy.SuccessOnAll(synchronise=False))    

    ### we create a sequence node called "Topics2BB" and a selector node called "Priorities"
    topics2BB = pt.composites.Sequence("Topics2BB", memory=False)
    priorities = pt.composites.Selector("Priorities", memory=False)

    ### we create an "Idle" node, which is a running node to keep the robot idle
    idle = pt.behaviours.Running(name="Idle")
    
    """
    TODO:  The first and second level of the tree structure is defined above, but please
    define the rest of the tree structure.

    Class definitions for your behaviours are provided in behaviours.py; you also need to fill out
    the behaviour implementations!

    HINT: Some behaviours from pt.behaviours may be useful to use as well.
    """

    #sstart behaviours
    battery_monitor = BatteryStatus2bb(topic_name="/battery_voltage")
    scan_monitor = LaserScan2bb(topic_name="/scan", safe_range=0.2)
    rotate = Rotate(name="Rotate", topic_name="/cmd_vel", ang_vel=0.5)
    stop_motion = StopMotion2bb(name="Stop Motion", topic_name="/cmd_vel")

    #Topics2BB: Monitor the battery and the laser scan
    topics2BB.add_children([battery_monitor, scan_monitor])

    #priority selector
    # Check battery first, if low ->rotate, if collision detected ->stop
    check_battery_low = pt.decorators.Condition(
        child=battery_monitor,
        name="Low Battery?",
        condition=operator.attrgetter("battery_low_warning"),
        value=True
    )

    detect_obstacle = pt.decorators.Condition(
        child=scan_monitor,
        name="Obstacle Near?",
        condition=operator.attrgetter("collision_detected"),
        value=True
    )

    stop_if_needed = pt.composites.Selector(name="Stop if Needed")
    stop_if_needed.add_children([detect_obstacle, stop_motion])

    rotate_if_safe = pt.composites.Sequence(name="Rotate if Safe")
    rotate_if_safe.add_children([pt.decorators.Inverter(detect_obstacle), rotate])

    priorities.add_children([stop_if_needed, check_battery_low, rotate_if_safe, idle])


    # TODO: construct the behaviour tree structure using the nodes and behaviours defined above
    # HINT: for reference, the sample tree structure in the README.md file might be useful

    root.add_children([topics2BB, priorities])

    # YOUR CODE HERE
#     raise NotImplementedError()

    return root

def main():
    """Initialises and executes the behaviour tree
    """
    rclpy.init(args=None)

    root = create_root()
    tree = ptr.trees.BehaviourTree(root=root, unicode_tree_debug=True)

    try:
        tree.setup(timeout=30.0)
    except ptr.exceptions.TimedOutError as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # user-initiated shutdown
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    
    # frequency of ticks
    tree.tick_tock(period_ms=100)    

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()