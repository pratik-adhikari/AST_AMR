#!/usr/bin/env python3

import functools
import py_trees as pt
import py_trees_ros as ptr
import py_trees.console as console
from sensor_msgs.msg import LaserScan
import rclpy
import sys
from autonomous_map_navigate.behaviors import *


def create_root() -> pt.behaviour.Behaviour:
    """
    Method to structure the behavior tree to monitor battery status and start rotation if battery is low.
    Also, the robot will stop if it detects an obstacle in front of it.
    
    The "collison_avoidance" behavior tree extends the "battery_monitor" behavior tree by adding a new feature
    to avoid collison with obstacles. Whenever the robot is about to collide with an object, the robot will
    automatically stop, overriding the input commands. The robot can be controlled either by joystick,
    where the command is published on the '/joy' topic or by command that is published on '/cmd_vel' topic.
    The laser scan data will be stored in blackboard by reading '/scan' topic. When an obstacle
    is detected, the 'stop_motion' behavior will be executed. The stop_motion behavior is prioritized over
    the rotate behavior.
    """

    ## define nodes and behaviors

    # define root node
    root = pt.composites.Parallel(
        name="root",
        policy=pt.common.ParallelPolicy.SuccessOnAll(
            synchronise=False
        )
    )    

    """
    Create a sequence node called "Topics2BB" and a selector node called "Priorities"    
    """
    ### YOUR CODE HERE ###
    topics2BB = pt.composites.Sequence("Topics2BB", memory=True)
    priorities = pt.composites.Selector("Priorities")

    """
    Using the battery_status2bb class, create a node called "Battery2BB" which subscribes to the topic "/battery_voltage"
    and the laser_scan_2bb class, create a node called "LaserScan2BB" which subscribes to the topic "/scan"
    """    
    ### YOUR CODE HERE ###
    battery2bb = battery_status2bb()
    LaserScan2BB = laser_scan_2bb()

    """
    Using the rotate class, create a node called "rotate_platform", and using the stop_motion class, create a node called "stop_platform"
    """
    ### YOUR CODE HERE ###
    rotate_platform = rotate()
    stop_platform = stop_motion()



    """
    Read the 'battery_low_warning' and 'collison_warning' from the blackboard and set a decorator node called "Battery Low?" to check if the battery is low 
    and "Colliding?" to check if any obstacle is within minimum distance.
    Please refer to the py_trees documentation for more information on decorators.
    """
    ### YOUR CODE HERE ###
    def check_battery_low_on_blackboard(blackboard: pt.blackboard.Blackboard) -> bool:
        return blackboard.battery_low_warning

    battery_emergency = pt.decorators.EternalGuard(
        name="Battery Low?",
        condition=check_battery_low_on_blackboard,
        blackboard_keys={"battery_low_warning"},
        child=rotate_platform
    )
    def check_collison_on_blackboard(blackboard: pt.blackboard.Blackboard) -> bool:
        return blackboard.collison_warning

    collison_check = pt.decorators.EternalGuard(
        name="Colliding?",
        condition=check_collison_on_blackboard,
        blackboard_keys={"collison_warning"},
        child=stop_platform
    )


    idle = pt.behaviours.Running(name="Idle")

    """
    Construct the behavior tree structure using the nodes and behaviors defined above
    """

    ### YOUR CODE HERE ###
    root.add_child(topics2BB)
    topics2BB.add_child(battery2bb)
    topics2BB.add_child(LaserScan2BB)
    root.add_child(priorities)
    priorities.add_child(collison_check)
    priorities.add_child(battery_emergency)
    priorities.add_child(idle)

    return root

def main():
    """
    Main function initiates behavior tree construction
    """
    rclpy.init(args=None)
    # Initializing the node with name "behavior_tree"
    root = create_root()
    tree = ptr.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
        )

    # setup the tree
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
    tree.tick_tock(period_ms=10)    
    
    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()

