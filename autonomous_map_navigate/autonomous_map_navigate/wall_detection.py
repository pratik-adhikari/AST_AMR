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
    is detected, the 'stop_motion' behavior will be executed. The stop_motion behavor is prioritized over
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
    topics2BB = pt.composites.Sequence("Topics2BB")
    priorities = pt.composites.Selector("Priorities")
    outOfBoundsRobot = pt.composites.Selector("OutOfBoundsRobot")
    distance_more_than_the_desired = pt.composites.Selector('distance_more_than_the_desired')
    
    

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
    aligning_to_wall = align_to_wall()
    follow_wall = forward_motion()
    without_wall_out_of_bounds = out_of_bounds_without_wall_class()
    with_wall_out_of_bounds = out_of_bounds_with_wall_class()
    resolve_direction = resolve_direction_class()
    # resolve_distance = resolve_distance_class()
    resolve_distance = resolving_distance()



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

    def check_wall_detection_on_blackboard(blackboard: pt.blackboard.Blackboard) -> bool:
        return blackboard.wall_detected_warning

    robot_alignment = pt.decorators.EternalGuard(
        name="Wall Detected?",
        condition=check_wall_detection_on_blackboard,
        blackboard_keys={"wall_detected_warning"},
        child=aligning_to_wall
    )

    def check_wall_following_on_blackboard(blackboard: pt.blackboard.Blackboard) -> bool:
        return blackboard.moving_along_wall

    moving_along_the_wall = pt.decorators.EternalGuard(
        name="Following Wall",
        condition=check_wall_following_on_blackboard,
        blackboard_keys={"moving_along_wall"},
        child=follow_wall
    )

    def check_out_of_bounds_without_wall(blackboard: pt.blackboard.Blackboard) -> bool:
        return blackboard.out_of_bounds_without_wall

    out_of_the_bounds_without_wall = pt.decorators.EternalGuard(
        name="Out of Bounds without Wall?",
        condition=check_out_of_bounds_without_wall,
        blackboard_keys={"out_of_bounds_without_wall"},
        child=without_wall_out_of_bounds
    )
    
    def check_out_of_bounds_with_wall(blackboard: pt.blackboard.Blackboard) -> bool:
        return blackboard.out_of_bounds_with_wall

    out_of_the_bounds_with_wall = pt.decorators.EternalGuard(
        name="Out of Bounds with Wall?",
        condition=check_out_of_bounds_with_wall,
        blackboard_keys={"out_of_bounds_with_wall"},
        child=with_wall_out_of_bounds
    )
    
    def check_distance_resolving(blackboard: pt.blackboard.Blackboard) -> bool:
        return blackboard.rotate_towards_nearest_wall

    rotate_towards_the_nearest_wall = pt.decorators.EternalGuard(
        name="Distance more than Desired?",
        condition=check_distance_resolving,
        blackboard_keys={"rotate_towards_nearest_wall"},
        child=resolve_direction
    )
    def check_straight_motion(blackboard: pt.blackboard.Blackboard) -> bool:
        return blackboard.moving_forward

    moving_forward_direction = pt.decorators.EternalGuard(
        name="Moving Forward?",
        condition=check_straight_motion,
        blackboard_keys={"moving_forward"},
        child=resolve_distance
    )


    # def check_desired_wall(blackboard: pt.blackboard.Blackboard) -> bool:
    #     return blackboard.distance_more_than_desired

    # desired_wall_following = pt.decorators.EternalGuard(
    #     name="Rotation towards Desired Wall?",
    #     condition=check_desired_wall,
    #     blackboard_keys={"distance_more_than_desired"},
    #     child=resolve_direction
    # )

    # def check_approaching_desired_wall(blackboard: pt.blackboard.Blackboard) -> bool:
    #     return blackboard.approaching_desired_wall

    # desired_wall_approach = pt.decorators.EternalGuard(
    #     name="Approaching towards Desired Wall?",
    #     condition=check_approaching_desired_wall,
    #     blackboard_keys={"approaching_desired_wall"},
    #     child=resolve_distance
    # )

    idle = pt.behaviours.Running(name="Idle")

    """
    construct the behvior tree structure using the nodes and behaviors defined above
    """

    ### YOUR CODE HERE ###
    root.add_child(topics2BB)
    topics2BB.add_child(battery2bb)
    topics2BB.add_child(LaserScan2BB)
    root.add_child(priorities)
    priorities.add_child(collison_check)
    priorities.add_child(battery_emergency)
    priorities.add_child(outOfBoundsRobot)
    priorities.add_child(robot_alignment)
    priorities.add_child(moving_along_the_wall)
    priorities.add_child(distance_more_than_the_desired)
    priorities.add_child(idle)
    outOfBoundsRobot.add_child(out_of_the_bounds_without_wall)
    outOfBoundsRobot.add_child(out_of_the_bounds_with_wall)
    distance_more_than_the_desired.add_child(rotate_towards_the_nearest_wall)
    distance_more_than_the_desired.add_child(moving_forward_direction)
    

    return root

def main():
    """
    Main function initiates behavior tree construction
    """
    rclpy.init(args=None)
    # Initialising the node with name "behavior_tree"
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