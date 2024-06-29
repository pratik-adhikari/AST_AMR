#!/usr/bin/env python3

import time
import py_trees as pt
import py_trees_ros as ptr
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy
import math
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from autonomous_map_navigate.utilities import *


class rotate(pt.behaviour.Behaviour):

    """
    Rotates the robot about z-axis 
    """

    def __init__(self, name="rotate platform", topic_name="/cmd_vel", direction=1, max_ang_vel=1.0):

        # self.logger.info("[ROTATE] initialising rotate behavior")

        # Set up topic name to publish rotation commands
        self.topic_name = topic_name

        # Set up Maximum allowable rotational velocity
        self.max_ang_vel = max_ang_vel # units: rad/sec

        # Set up direction of rotation
        self.direction = direction

        # Execution checker
        self.sent_goal = False

        # become a behaviour
        super(rotate, self).__init__(name)

    def setup(self, **kwargs):
        """
        Setting up things which generally might require time to prevent delay in tree initialisation
        """
        self.logger.info("[ROTATE] setting up rotate behavior")
        
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        # Create publisher to publish rotation commands
        self.cmd_vel_pub = self.node.create_publisher(
            msg_type=Twist,
            topic=self.topic_name,
            qos_profile=ptr.utilities.qos_profile_latched()
        )

        self.feedback_message = "setup"
        return True

    def update(self):
        """
        Primary function of the behavior is implemented in this method

        Rotating the robot at maximum allowed angular velocity in a given direction, 
        where, if **direction** is +1, it implies clockwise rotation, and if it is -1, it implies
        counter-clockwise rotation
        """
        self.logger.info("[ROTATE] update: updating rotate behavior")
        self.logger.debug("%s.update()" % self.__class__.__name__)

        # Send the rotation command to self.topic_name in this method using the message type Twist()
        
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.
        twist_msg.angular.z = 0.1
                    
        self.cmd_vel_pub.publish(twist_msg)
        return pt.common.Status.RUNNING

    def terminate(self, new_status):
        """
        terminate() is trigerred once the execution of the behavior finishes, 
        i.e. when the status changes from RUNNING to SUCCESS or FAILURE
        """
        self.logger.info("[ROTATE] terminate: publishing zero angular velocity")
        twist_msg = Twist()
        twist_msg.linear.x = 0.
        twist_msg.linear.y = 0.
        twist_msg.angular.z = 0.
                    
        self.cmd_vel_pub.publish(twist_msg)
        self.sent_goal = False
        return super().terminate(new_status)
class resolving_distance(pt.behaviour.Behaviour):

    """
    Once the direction is resolved when the robot is away more than 0.6 units away from the wall, this class
    resolves distance by moving forward.
    """

    def __init__(self, name="resolving distance", topic_name="/cmd_vel", direction=1, max_ang_vel=1.0):

        # self.logger.info("[ROTATE] initialising rotate behavior")

        # Set up topic name to publish rotation commands
        self.topic_name = topic_name

        # Set up Maximum allowable rotational velocity
        self.max_ang_vel = max_ang_vel # units: rad/sec

        # Set up direction of rotation
        self.direction = direction

        # Execution checker
        self.sent_goal = False

        # become a behaviour
        super(resolving_distance, self).__init__(name)

    def setup(self, **kwargs):
        """
        Setting up things which generally might require time to prevent delay in tree initialisation
        """
        self.logger.info("[ROTATE] setting up rotate behavior")
        
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        # Create publisher to publish rotation commands
        self.cmd_vel_pub = self.node.create_publisher(
            msg_type=Twist,
            topic=self.topic_name,
            qos_profile=ptr.utilities.qos_profile_latched()
        )

        self.feedback_message = "setup"
        return True

    def update(self):
        """
        Simple forward motion commands. 
        """
        self.logger.info("[ROTATE] update: updating rotate behavior")
        self.logger.debug("%s.update()" % self.__class__.__name__)

        # Send the rotation command to self.topic_name in this method using the message type Twist()
        
        twist_msg = Twist()
        twist_msg.linear.x = 0.1
        twist_msg.linear.y = 0.
        twist_msg.angular.z = 0.0
                    
        self.cmd_vel_pub.publish(twist_msg)
        return pt.common.Status.RUNNING

    def terminate(self, new_status):
        """
        terminate() is trigerred once the execution of the behavior finishes, 
        i.e. when the status changes from RUNNING to SUCCESS or FAILURE
        """
        self.logger.info("[ROTATE] terminate: publishing zero angular velocity")
        twist_msg = Twist()
        twist_msg.linear.x = 0.
        twist_msg.linear.y = 0.
        twist_msg.angular.z = 0.
                    
        self.cmd_vel_pub.publish(twist_msg)
        self.sent_goal = False
        return super().terminate(new_status)
class forward_motion(pt.behaviour.Behaviour):
    """
    Moving robot along the wall when the robot is detecting and aligned with the wall
    """

    def __init__(self, 
                 name: str="following wall", 
                 topic_name1: str="/cmd_vel", 
                 topic_name2: str="/joy"):
        super(forward_motion, self).__init__(name)
        # Set up topic name to publish forward commands
        self.cmd_vel_topic = topic_name1
        self.joy_topic = topic_name2
        self.blackboard = pt.blackboard.Blackboard()

    def setup(self, **kwargs):
        """
        Setting up things which generally might require time to prevent delay in tree initialisation
        """
        self.logger.info("[MOVING ALONG THE WALL] setting up moving along wall behavior")

        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        # Create publisher to publish forward commands
        self.cmd_vel_pub = self.node.create_publisher(
            msg_type=Twist,
            topic=self.cmd_vel_topic,
            qos_profile=ptr.utilities.qos_profile_latched()
        )

        # Create publisher to override joystick commands
        self.joy_pub = self.node.create_publisher(
            msg_type=Joy,
            topic=self.joy_topic,
            qos_profile=ptr.utilities.qos_profile_latched()
        )
        
        self.feedback_message = "setup"
        return True

    def update(self):
        """
        Primary function of the behavior is implemented in this method

        Rotating the robot at maximum allowed angular velocity in a given direction, 
        where if _direction_ is +1, it implies clockwise rotation, and if it is -1, it implies
        counter-clockwise rotation
        """
        self.logger.info("[MOVING FORWARD] update: updating align to wall behavior")
        self.logger.debug("%s.update()" % self.__class__.__name__)

        """
        Send the zero rotation command to self.cmd_vel_topic
        """
        
        ## YOUR CODE HERE ##
    
        # Example inputs
        

        self.logger.info("[MOVING FORWARD] update: MOVING FORWARD BEHAVIOR when wall detected")

        twist_msg = Twist()
        self.logger.info("moving forward")
        twist_msg.linear.x = 0.1
        twist_msg.linear.y = 0.
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)

        """
        sending self.joy_pub in this method. The frame_id for Joy() message is
        "/dev/input/js0". It is similar to just pressing the deadman button on the joystick.
        Nothing to implement here.
        """
        ## Uncomment the following lines to publish Joy() message when running on the robot ##
        # joyMessage = Joy()
        # joyMessage.header.frame_id = "/dev/input/js0"
        # joyMessage.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # joyMessage.buttons = [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
        # self.joy_pub.publish(joyMessage)        

        return pt.common.Status.RUNNING


    def terminate(self, new_status):
        """
        terminate() is trigerred once the execution of the behavior finishes, 
        i.e. when the status changes from RUNNING to SUCCESS or FAILURE
        """
        self.logger.info("[MOVING FORWARD] terminate: publishing zero angular velocity after aligning")
        twist_msg = Twist()
        twist_msg.linear.x = 0.
        twist_msg.linear.y = 0.
        twist_msg.angular.z = 0.
        self.cmd_vel_pub.publish(twist_msg)
        self.sent_goal = False
        return super().terminate(new_status)

class out_of_bounds_with_wall_class(pt.behaviour.Behaviour):
    """
    Trigger class when the robot is not sensing a wall or line but senses a wall position(left or right) obtained previously.
    """

    def __init__(self, 
                 name: str="Out of Bounds with Wall", 
                 topic_name1: str="/cmd_vel", 
                 topic_name2: str="/joy"):
        super(out_of_bounds_with_wall_class, self).__init__(name)
        # Set up topic name to publish forward commands
        self.cmd_vel_topic = topic_name1
        self.joy_topic = topic_name2
        self.blackboard = pt.blackboard.Blackboard()

    def setup(self, **kwargs):
        """
        Setting up things which generally might require time to prevent delay in tree initialisation
        """
        self.logger.info("[MOVING ALONG THE WALL] setting up moving along wall behavior")

        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        # Create publisher to publish forward commands
        self.cmd_vel_pub = self.node.create_publisher(
            msg_type=Twist,
            topic=self.cmd_vel_topic,
            qos_profile=ptr.utilities.qos_profile_latched()
        )

        # Create publisher to override joystick commands
        self.joy_pub = self.node.create_publisher(
            msg_type=Joy,
            topic=self.joy_topic,
            qos_profile=ptr.utilities.qos_profile_latched()
        )
        self.feedback_message = "setup"
        return True

    def update(self):
        """
        Primary function of the behavior is implemented in this method

        Rotating the robot at maximum allowed angular velocity in a given direction, 
        where if _direction_ is +1, it implies clockwise rotation, and if it is -1, it implies
        counter-clockwise rotation
        """
        self.logger.info("[MOVING FORWARD] update: updating align to wall behavior")
        self.logger.debug(f"{self.__class__.__name__}.update()")

        """
        Send the zero rotation command to self.cmd_vel_topic
        """

        ## YOUR CODE HERE ##

        
        
        point_at_min_dist = self.blackboard.get('point_at_min_dist')
        freeze_laser_scan = self.blackboard.get('freeze_laser_scan')
        slope = self.blackboard.get('slope')
        laser_scan = self.blackboard.get('laser_scan')
        mid_idx = int((len(laser_scan)-1)/2)
        wall_position = self.blackboard.get('wall_position_out_of_bounds')
        
            

        if slope > -0.1 and slope < 0.1 and freeze_laser_scan == False:
            self.logger.info("[out_of_bounds_robot] update: Move to Terminate when slope is zero")
            self.blackboard.set('distance_more_than_desired',True)
            self.blackboard.set('back_inside_the_bounds',True)
            self.blackboard.set('moving_along_wall',False)
            if np.isclose(self.blackboard.get('point_at_min_distFreeze'), 
                                self.blackboard.get('average_mid_point'), atol=0.2) == True:
                self.blackboard.set('moving_forward',True)
                self.blackboard.set('distance_more_than_desired',False)
                self.blackboard.set('point_at_min_distFreeze_Flag',False)
                
            else:
                self.blackboard.set('moving_along_wall',False)
                self.blackboard.set('distance_more_than_desired',True)
                self.blackboard.set('rotate_towards_nearest_wall',True)
                self.blackboard.set('point_at_min_distFreeze_Flag',True)
                self.blackboard.set('moving_forward',False)
                self.blackboard.set('wall_detected_warning',False)

            return pt.common.Status.SUCCESS
        elif freeze_laser_scan == True:
            if wall_position == 'left':    
                    twist_msg = Twist()
                    self.logger.info("turning right after reaching 2.5")
                    twist_msg.linear.x = 0.1
                    twist_msg.linear.y = 0.
                    twist_msg.angular.z = 0.1
                    self.cmd_vel_pub.publish(twist_msg)
                    return pt.common.Status.RUNNING
                    
            elif wall_position == 'right':
                    twist_msg = Twist()
                    self.logger.info("turning right after reaching 2.5")
                    twist_msg.linear.x = 0.1
                    twist_msg.linear.y = 0.
                    twist_msg.angular.z = -0.1
                    self.cmd_vel_pub.publish(twist_msg)
                    return pt.common.Status.RUNNING
            return pt.common.Status.RUNNING
            
        else:
            self.blackboard.set('moving_along_wall',False)
            twist_msg = Twist()
            self.logger.info("moving forward")
            twist_msg.linear.x = 0.1
            twist_msg.linear.y = 0.
            twist_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(twist_msg)
            return pt.common.Status.RUNNING

        """
        sending self.joy_pub in this method. The frame_id for Joy() message is
        "/dev/input/js0". It is similar to just pressing the deadman button on the joystick.
        Nothing to implement here.
        """
        ## Uncomment the following lines to publish Joy() message when running on the robot ##
        # joyMessage = Joy()
        # joyMessage.header.frame_id = "/dev/input/js0"
        # joyMessage.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # joyMessage.buttons = [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
        # self.joy_pub.publish(joyMessage)        

        return pt.common.Status.RUNNING
    def terminate(self, new_status):
        """
        terminate() is trigerred once the execution of the behavior finishes, 
        i.e. when the status changes from RUNNING to SUCCESS or FAILURE
        """
        self.logger.info("[out_of_bounds_with_wall_class] terminate: publishing zero angular velocity after aligning")
        # time.sleep(1)
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)
        self.sent_goal = False
        return super().terminate(new_status)

class out_of_bounds_without_wall_class(pt.behaviour.Behaviour):
    """
    When the robot is not sensing the wall or points from the laser scan data. 
    """

    def __init__(self, 
                 name: str="Out of Bounds without Wall?", 
                 topic_name1: str="/cmd_vel", 
                 topic_name2: str="/joy"):
        super(out_of_bounds_without_wall_class, self).__init__(name)
        # Set up topic name to publish forward commands
        self.cmd_vel_topic = topic_name1
        self.joy_topic = topic_name2
        self.blackboard = pt.blackboard.Blackboard()

    def setup(self, **kwargs):
        """
        Setting up things which generally might require time to prevent delay in tree initialisation
        """
        self.logger.info("[MOVING ALONG THE WALL] setting up moving along wall behavior")

        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        # Create publisher to publish forward commands
        self.cmd_vel_pub = self.node.create_publisher(
            msg_type=Twist,
            topic=self.cmd_vel_topic,
            qos_profile=ptr.utilities.qos_profile_latched()
        )

        # Create publisher to override joystick commands
        self.joy_pub = self.node.create_publisher(
            msg_type=Joy,
            topic=self.joy_topic,
            qos_profile=ptr.utilities.qos_profile_latched()
        )
        self.feedback_message = "setup"
        return True

    def update(self):
        """
        Primary function of the behavior is implemented in this method

        Rotating the robot at maximum allowed angular velocity in a given direction, 
        where if _direction_ is +1, it implies clockwise rotation, and if it is -1, it implies
        counter-clockwise rotation
        """
        self.logger.info("[MOVING FORWARD] update: updating align to wall behavior")
        self.logger.debug("%s.update()" % self.__class__.__name__)

        """
        Send the zero rotation command to self.cmd_vel_topic
        """
        
        ## YOUR CODE HERE ##
    
        

        self.logger.info("[out_of_bounds_robot] update: resolving the closed corner when wall NOT detected")


        
        angle = 180
        def rotate(angle):
            time_for_rotation = abs(angle) / 0.5
            twist_msg = Twist()
            self.logger.info("moving forward")
            twist_msg.linear.x = 0.0
            twist_msg.linear.y = 0.
            twist_msg.angular.z = 0.2
            self.cmd_vel_pub.publish(twist_msg)
            time.sleep(time_for_rotation)
            
            
        def forward():
            twist_msg = Twist()
            self.logger.info("moving forward")
            twist_msg.linear.x = 0.25
            twist_msg.linear.y = 0.
            twist_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(twist_msg)
            
        



        

        """
        sending self.joy_pub in this method. The frame_id for Joy() message is
        "/dev/input/js0". It is similar to just pressing the deadman button on the joystick.
        Nothing to implement here.
        """
        ## Uncomment the following lines to publish Joy() message when running on the robot ##
        # joyMessage = Joy()
        # joyMessage.header.frame_id = "/dev/input/js0"
        # joyMessage.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # joyMessage.buttons = [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
        # self.joy_pub.publish(joyMessage)        

        return pt.common.Status.SUCCESS


    def terminate(self, new_status):
        """
        terminate() is trigerred once the execution of the behavior finishes, 
        i.e. when the status changes from RUNNING to SUCCESS or FAILURE
        """
        self.logger.info("[ALIGN] terminate: publishing zero angular velocity after aligning")
        # time.sleep(1)
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)
        self.sent_goal = False
        return super().terminate(new_status)

class stop_motion(pt.behaviour.Behaviour):

    """
    Stops the robot when it is controlled using joystick or by cmd_vel command
    """

    def __init__(self, 
                 name: str="stop platform", 
                 topic_name1: str="/cmd_vel", 
                 topic_name2: str="/joy"):
        super(stop_motion, self).__init__(name)
        # Set up topic name to publish rotation commands
        self.cmd_vel_topic = topic_name1
        self.joy_topic = topic_name2

    def setup(self, **kwargs):
        """
        Setting up things which generally might require time to prevent delay in tree initialisation
        """
        self.logger.info("[STOP MOTION] setting up stop motion behavior")

        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        # Create publisher to publish rotation commands
        self.cmd_vel_pub = self.node.create_publisher(
            msg_type=Twist,
            topic=self.cmd_vel_topic,
            qos_profile=ptr.utilities.qos_profile_latched()
        )

        # Create publisher to override joystick commands
        self.joy_pub = self.node.create_publisher(
            msg_type=Joy,
            topic=self.joy_topic,
            qos_profile=ptr.utilities.qos_profile_latched()
        )
        
        self.feedback_message = "setup"
        return True

    def update(self):
        """
        Primary function of the behavior is implemented in this method

        Rotating the robot at maximum allowed angular velocity in a given direction, 
        where if _direction_ is +1, it implies clockwise rotation, and if it is -1, it implies
        counter-clockwise rotation
        """
        self.logger.info("[STOP] update: updating stop behavior")
        self.logger.debug("%s.update()" % self.__class__.__name__)

        """
        Send the zero rotation command to self.cmd_vel_topic
        """
        
        ## YOUR CODE HERE ##
        
        twist_msg = Twist()
        twist_msg.linear.x = 0.
        twist_msg.linear.y = 0.
        twist_msg.angular.z = 0.
        self.cmd_vel_pub.publish(twist_msg)


        """
        sending self.joy_pub in this method. The frame_id for Joy() message is
        "/dev/input/js0". It is similar to just pressing the deadman button on the joystick.
        Nothing to implement here.
        """
        ## Uncomment the following lines to publish Joy() message when running on the robot ##
        joyMessage = Joy()
        joyMessage.header.frame_id = "/dev/input/js0"
        joyMessage.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        joyMessage.buttons = [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
        self.joy_pub.publish(joyMessage)        

        return pt.common.Status.SUCCESS


    def terminate(self, new_status):
        """
        terminate() is trigerred once the execution of the behavior finishes, 
        i.e. when the status changes from RUNNING to SUCCESS or FAILURE
        """
        self.logger.info("[ROTATE] terminate: publishing zero angular velocity")
        twist_msg = Twist()
        twist_msg.linear.x = 0.
        twist_msg.linear.y = 0.
        twist_msg.angular.z = 0.
                    
        self.cmd_vel_pub.publish(twist_msg)
        self.sent_goal = False
        return pt.common.Status.SUCCESS

class align_to_wall(pt.behaviour.Behaviour):
    """
    Align the robot when the robot is detecting the wall and is within 0.6 distance away from the wall.
    """

    def __init__(self, 
                 name: str="align robot", 
                 topic_name1: str="/cmd_vel", 
                 topic_name2: str="/joy"):
        super(align_to_wall, self).__init__(name)
        # Set up topic name to publish rotation commands
        self.cmd_vel_topic = topic_name1
        self.joy_topic = topic_name2
        self.blackboard = pt.blackboard.Blackboard()

    def setup(self, **kwargs):
        """
        Setting up things which generally might require time to prevent delay in tree initialisation
        """
        self.logger.info("[ALIGN TO WALL] setting up aligning motion behavior")

        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        # Create publisher to publish rotation commands
        self.cmd_vel_pub = self.node.create_publisher(
            msg_type=Twist,
            topic=self.cmd_vel_topic,
            qos_profile=ptr.utilities.qos_profile_latched()
        )

        # Create publisher to override joystick commands
        self.joy_pub = self.node.create_publisher(
            msg_type=Joy,
            topic=self.joy_topic,
            qos_profile=ptr.utilities.qos_profile_latched()
        )
        self.feedback_message = "setup"
        return True

    def update(self):
        """
        Primary function of the behavior is implemented in this method

        Rotating the robot at maximum allowed angular velocity in a given direction, 
        where if _direction_ is +1, it implies clockwise rotation, and if it is -1, it implies
        counter-clockwise rotation
        """
        self.logger.info("[ALIGNING] update: updating align to wall behavior")
        self.logger.debug("%s.update()" % self.__class__.__name__)

        """
        Send the zero rotation command to self.cmd_vel_topic
        """
        
        ## YOUR CODE HERE ##
    
        # Example inputs
        slope = self.blackboard.get('slope')
        slopeForFreeze = self.blackboard.get('slopeForFreeze')
        distance = self.blackboard.get('distance')


        self.logger.info("[ALIGN] update: before rotation")
        angle = np.arctan(slope)
        angularVel = angle / 180 * math.pi
        duration = angle / angularVel

        
        self.logger.info("[ALIGN] update: before rotation")
        
        laser_scan = self.blackboard.get('laser_scan')
        laser_scan_split = np.array_split(laser_scan, 2)
        if np.sum(laser_scan_split[0])>np.sum(laser_scan_split[1]):
            self.blackboard.set('wall_position','left')
        else:
            self.blackboard.set('wall_position','right')
        if np.sum(laser_scan_split[0])>np.sum(laser_scan_split[1]):
            self.blackboard.set('wall_position','left')
        else:
            self.blackboard.set('wall_position','right')
            
        
        
        
        if self.blackboard.get('average_mid_point_for_corner') <0.6 and (slopeForFreeze > 0.05 and slopeForFreeze < -0.05):
            if slope > -0.04 and slope < 0.04:
                self.logger.info("[ALIGN] update: TERMINATING THE ALIGNING BEHAVIOR when slope is zero")
                return pt.common.Status.SUCCESS

            elif slope<-0.01:
                twist_msg = Twist()
                twist_msg.linear.x = 0.0
                twist_msg.linear.y = 0.
                self.logger.info("[ALIGN] update: aligning to the wall when slope is not zero")
                twist_msg.angular.z = -0.1
                self.cmd_vel_pub.publish(twist_msg)
                return pt.common.Status.RUNNING
        
            elif slope>0.01:
                twist_msg = Twist()
                twist_msg.linear.x = 0.0
                twist_msg.linear.y = 0.
                self.logger.info("[ALIGN] update: aligning to the wall when slope is not zero")
                twist_msg.angular.z = 0.1
                self.cmd_vel_pub.publish(twist_msg)
                return pt.common.Status.RUNNING
        else:
            if slope > -0.04 and slope < 0.04:
                self.logger.info("[ALIGN] update: TERMINATING THE ALIGNING BEHAVIOR when slope is zero")
                return pt.common.Status.SUCCESS

            elif slope<-0.01:
                twist_msg = Twist()
                twist_msg.linear.x = 0.05
                twist_msg.linear.y = 0.
                self.logger.info("[ALIGN] update: aligning to the wall when slope is not zero")
                twist_msg.angular.z = -0.1
                self.cmd_vel_pub.publish(twist_msg)
                return pt.common.Status.RUNNING
        
            elif slope>0.01:
                twist_msg = Twist()
                twist_msg.linear.x = 0.05
                twist_msg.linear.y = 0.
                self.logger.info("[ALIGN] update: aligning to the wall when slope is not zero")
                twist_msg.angular.z = 0.1
                self.cmd_vel_pub.publish(twist_msg)
                return pt.common.Status.RUNNING
            
            
        
        """
        sending self.joy_pub in this method. The frame_id for Joy() message is
        "/dev/input/js0". It is similar to just pressing the deadman button on the joystick.
        Nothing to implement here.
        """
        # Uncomment the following lines to publish Joy() message when running on the robot ##
        # joyMessage = Joy()
        # joyMessage.header.frame_id = "/dev/input/js0"
        # joyMessage.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # joyMessage.buttons = [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
        # self.joy_pub.publish(joyMessage)        

        return pt.common.Status.RUNNING


    def terminate(self, new_status):
        """
        terminate() is trigerred once the execution of the behavior finishes, 
        i.e. when the status changes from RUNNING to SUCCESS or FAILURE
        """
        self.logger.info("[ALIGN] terminate: publishing zero angular velocity after aligning")
        twist_msg = Twist()
        twist_msg.linear.x = 0.
        twist_msg.linear.y = 0.
        twist_msg.angular.z = 0.
        self.cmd_vel_pub.publish(twist_msg)
        self.sent_goal = False
        return super().terminate(new_status)
class resolve_direction_class(pt.behaviour.Behaviour):
    """
    Rotate the robot towards the wall with the minimum distance from the laser/robot.
    """

    def __init__(self, 
                 name: str="resolving direction", 
                 topic_name1: str="/cmd_vel", 
                 topic_name2: str="/joy"):
        super(resolve_direction_class, self).__init__(name)
        # Set up topic name to publish forward commands
        self.cmd_vel_topic = topic_name1
        self.joy_topic = topic_name2
        self.blackboard = pt.blackboard.Blackboard()

    def setup(self, **kwargs):
        """
        Setting up things which generally might require time to prevent delay in tree initialisation
        """
        self.logger.info("[MOVING ALONG THE WALL] setting up moving along wall behavior")

        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        # Create publisher to publish forward commands
        self.cmd_vel_pub = self.node.create_publisher(
            msg_type=Twist,
            topic=self.cmd_vel_topic,
            qos_profile=ptr.utilities.qos_profile_latched()
        )

        # Create publisher to override joystick commands
        self.joy_pub = self.node.create_publisher(
            msg_type=Joy,
            topic=self.joy_topic,
            qos_profile=ptr.utilities.qos_profile_latched()
        )
        
        self.feedback_message = "setup"
        return True

    def update(self):
        """
        Primary function of the behavior is implemented in this method

        Rotating the robot at maximum allowed angular velocity in a given direction, 
        where if _direction_ is +1, it implies clockwise rotation, and if it is -1, it implies
        counter-clockwise rotation
        """
        self.logger.info("[resolve_direction_class]] update: updating resolve direction behavior")
        self.logger.debug("%s.update()" % self.__class__.__name__)

        """
        Send the zero rotation command to self.cmd_vel_topic
        """
        
        ## YOUR CODE HERE ##
    
        # Example inputs
        # slope = self.blackboard.get('slope')
        point_at_min_distFreeze = self.blackboard.get('point_at_min_distFreeze')
        point_at_min_dist = self.blackboard.get('point_at_min_dist')
        average_mid_point = self.blackboard.get('average_mid_point')
        

        self.logger.info("[resolve_direction_class] update: resolve_direction BEHAVIOR when wall is away")

        
        if np.isclose(point_at_min_distFreeze, average_mid_point, atol=0.2) == True:
            self.logger.info("[resolve_direction_class] update: Rotating and stopping when the alignment towards nearest wall is complete")
            return pt.common.Status.SUCCESS

        else:
            if self.blackboard.get('wall_position') == 'left':    
                twist_msg = Twist()
                self.logger.info("[resolve_direction_class] turning right")
                twist_msg.linear.x = 0.05
                twist_msg.linear.y = 0.
                twist_msg.angular.z = 0.1
                self.cmd_vel_pub.publish(twist_msg)
            elif self.blackboard.get('wall_position') == 'right':
                twist_msg = Twist()
                self.logger.info("[resolve_direction_class] turning right")
                twist_msg.linear.x = 0.05
                twist_msg.linear.y = 0.
                twist_msg.angular.z = -0.1
                self.cmd_vel_pub.publish(twist_msg)
            return pt.common.Status.RUNNING

        """
        sending self.joy_pub in this method. The frame_id for Joy() message is
        "/dev/input/js0". It is similar to just pressing the deadman button on the joystick.
        Nothing to implement here.
        """
        ## Uncomment the following lines to publish Joy() message when running on the robot ##
        # joyMessage = Joy()
        # joyMessage.header.frame_id = "/dev/input/js0"
        # joyMessage.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # joyMessage.buttons = [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
        # self.joy_pub.publish(joyMessage)        

        return pt.common.Status.RUNNING


    def terminate(self, new_status):
        """
        terminate() is trigerred once the execution of the behavior finishes, 
        i.e. when the status changes from RUNNING to SUCCESS or FAILURE
        """
        self.logger.info("[resolve_distance_class] terminate: publishing zero angular velocity after aligning")
        twist_msg = Twist()
        twist_msg.linear.x = 0.
        twist_msg.linear.y = 0.
        twist_msg.angular.z = 0.
        self.cmd_vel_pub.publish(twist_msg)
        self.sent_goal = False
        return super().terminate(new_status)
    

class battery_status2bb(ptr.subscribers.ToBlackboard):

    """
    Checking battery status
    """
    def __init__(self, 
                 topic_name: str="/battery_voltage",
                 name: str=pt.common.Name.AUTO_GENERATED, 
                 threshold: float=30.0):
        super().__init__(name=name,
                        topic_name=topic_name,
                        topic_type=Float32,
                        blackboard_variables={'battery': 'data'},
                        initialise_variables={'battery': 50.0},
                        clearing_policy=pt.common.ClearingPolicy.NEVER,  # to decide when data should be cleared/reset.
                        qos_profile=ptr.utilities.qos_profile_unlatched()
                        )
        self.blackboard.register_key(
            key='battery_low_warning',
            access=pt.common.Access.WRITE
        ) 
        self.blackboard.battery_low_warning = False   # decision making
        self.threshold = threshold

    def update(self):
        """
        Primary function of the behavior is implemented in this method

        Call the parent to write the raw data to the blackboard and then check against the
        threshold to determine if the low warning flag should also be updated.
        """
        self.logger.info('[BATTERY] update: running batter_status2bb update')
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(battery_status2bb, self).update()
        
        """
        check battery voltage level stored in self.blackboard.battery. By comparing with threshold value, update the value of 
        self.blackboad.battery_low_warning
        """

        ## YOUR CODE HERE ##
        print('checking battery', self.blackboard.battery, self.threshold)
        if self.blackboard.battery < self.threshold:
            self.blackboard.battery_low_warning = True 
        else:
            self.blackboard.battery_low_warning = False 

        return pt.common.Status.SUCCESS

class laser_scan_2bb(ptr.subscribers.ToBlackboard):

    """
    Checking laser_scan to trigger and control all conditions for checking the above classes.
    """
    def __init__(self, 
                 topic_name: str="/scan",
                #  topic_name: str="/sick_lms_1xx/scan",
                 name: str=pt.common.Name.AUTO_GENERATED, 
                 safe_range: float=0.15,
                 colliding_range: float=1.5):
        super().__init__(name=name,
                        topic_name=topic_name,
                        topic_type=LaserScan,
                        blackboard_variables={'laser_scan':'ranges'},
                        clearing_policy=pt.common.ClearingPolicy.NEVER,  # to decide when data should be cleared/reset.
                        # qos_profile=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
                        qos_profile=QoSProfile(
                                    reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                                    history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                    depth=10
                                )
                        )
        self.blackboard.register_key(
            key='collison_warning',
            access=pt.common.Access.WRITE
        )
        self.blackboard.register_key(
            key='point_at_min_dist',
            access=pt.common.Access.WRITE
        )
        self.blackboard.register_key(
            key='desired_wall',
            access=pt.common.Access.WRITE
        )

        self.blackboard.register_key(
            key='wall_detected_warning',
            access=pt.common.Access.WRITE
        )

        self.blackboard.register_key(
            key='online_params',
            access=pt.common.Access.WRITE
        )

        self.blackboard.register_key(
            key='processed_data',
            access=pt.common.Access.WRITE
        )

        self.blackboard.register_key(
            key='distance',
            access=pt.common.Access.WRITE
        )

        self.blackboard.register_key(
            key='slope',
            access=pt.common.Access.WRITE
        )

        self.blackboard.register_key(
            key='intercept',
            access=pt.common.Access.WRITE
        )

        self.blackboard.register_key(
            key='end_pts',
            access=pt.common.Access.WRITE
        )

        self.blackboard.register_key(
            key='wall_position',
            access=pt.common.Access.WRITE
        )

        self.blackboard.register_key(
            key='moving_along_wall',
            access=pt.common.Access.WRITE
        )
        self.blackboard.register_key(
            key='out_of_the_bounds',
            access=pt.common.Access.WRITE
        )
        self.blackboard.register_key(
            key='wall_position',
            access=pt.common.Access.WRITE
        )
        self.blackboard.register_key(
            key='freeze_laser_scan',
            access=pt.common.Access.WRITE
        )
        self.blackboard.register_key(
            key='slopeForFreeze',
            access=pt.common.Access.WRITE
        )
        self.blackboard.register_key(
            key='out_of_bounds_with_wall',
            access=pt.common.Access.WRITE
        )
        self.blackboard.register_key(
            key='out_of_bounds_without_wall',
            access=pt.common.Access.WRITE
        )
        self.blackboard.register_key(
            key='distance_more_than_desired',
            access=pt.common.Access.WRITE
        )
        self.blackboard.register_key(
            key='average_mid_point',
            access=pt.common.Access.WRITE
        )
        self.blackboard.register_key(
            key='point_at_min_distFreeze_Flag',
            access=pt.common.Access.WRITE
        )
        self.blackboard.register_key(
            key='point_at_min_distFreeze',
            access=pt.common.Access.WRITE
        )
        self.blackboard.register_key(
        key='point_at_min_distFreeze',
            access=pt.common.Access.WRITE
        )
        self.blackboard.register_key(
        key='moving_forward',
            access=pt.common.Access.WRITE
        )
        self.blackboard.register_key(
        key='rotate_towards_nearest_wall',
            access=pt.common.Access.WRITE
        )
        self.blackboard.register_key(
        key='back_inside_the_bounds',
        access=pt.common.Access.WRITE
        )
        self.blackboard.register_key(
        key='average_mid_point_for_corner',
        access=pt.common.Access.WRITE
        )
        self.blackboard.register_key(
        key='wall_position_out_of_bounds',
        access=pt.common.Access.WRITE
        )


        self.blackboard.collison_warning = False
        self.blackboard.wall_detected_warning = False
        self.blackboard.wall_position = None
        self.safe_min_range = safe_range
        self.colliding_range = colliding_range
        self.blackboard.point_at_min_dist = 0.0
        self.blackboard.moving_along_wall =False 
        self.blackboard.out_of_the_bounds = False
        self.blackboard.freeze_laser_scan = False
        self.blackboard.out_of_bounds_with_wall = False
        self.blackboard.out_of_bounds_without_wall = False
        self.blackboard.distance_more_than_desired = False
        self.blackboard.average_mid_point = 0.0
        self.blackboard.point_at_min_distFreeze_Flag = False
        self.blackboard.rotate_towards_nearest_wall = False
        self.blackboard.moving_forward =  False
        self.blackboard.back_inside_the_bounds = False
        self.blackboard.average_mid_point_for_corner = 0.0
        self.blackboard.wall_position_out_of_bounds = None
        
        
        



    def update(self):
        """
        Primary function of the behavior is implemented in this method

        Call the parent to write the raw data to the blackboard and then check against the
        threshold to set the warning if the robot is close to any obstacle.
        """
        self.logger.info("[LASER SCAN] update: running laser_scan_2bb update")
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(laser_scan_2bb, self).update()

        """
        Based on the closeness of laser scan points (any of them) to robot, update the value of self.blackboard.collison_warning.
        Assign the minimum value of laser scan to self.blackboard.point_at_min_dist. 
        Note: The min() function can be used to find the minimum value in a list.
        """

        ## YOUR CODE HERE ##   
        if self.blackboard.exists('laser_scan'):
            self.logger.info("process initiated")

            laser_scan = self.blackboard.get('laser_scan')
            laser_scan = np.array(laser_scan)
            laser_scan_split = np.array_split(laser_scan, 5)
            leftend = laser_scan_split[0]
            leftmiddle = laser_scan_split[1]
            center = laser_scan_split[2]
            rightmiddle = laser_scan_split[3]
            rightend = laser_scan_split[4]
            
            
            point_min = min(laser_scan)
            point_min_idx  = np.where(laser_scan == point_min)[0]
            
        
            temp = np.array(self.blackboard.laser_scan)
            self.blackboard.desired_wall = temp
            temp[temp <= 0.05] = 1.1
            temp = np.nan_to_num(temp, nan=5)
            min_of_temp = min(temp)
            point_min_temp_idx = np.where(temp == min_of_temp)[0]
            max_angle= np.deg2rad(90)
            min_angle= np.deg2rad(-90)
            max_range= 2.0
            min_range= 0.05
            sigma= 0.2
            rf_max_pts= 10
            reduce_bool= True
            processed_data = process_data(range_data= temp, max_angle=max_angle,
                                min_angle= min_angle, max_range= max_range,
                                min_range=min_range, sigma=sigma,
                                rf_max_pts=rf_max_pts, reduce_bool= reduce_bool)  
            

            self.blackboard.point_at_min_dist = min(temp)
            if self.blackboard.point_at_min_distFreeze_Flag == False:
                self.blackboard.point_at_min_distFreeze = min(temp)
            mid_idx = int((len(laser_scan)-1)/2)
            if self.blackboard.out_of_the_bounds == False:
                if point_min_idx[0] <= mid_idx:
                    self.logger.info("[ALIGN] update: out of bounds, rotating to the right")
                    self.blackboard.wall_position = 'right'
                elif point_min_idx[0] > mid_idx:
                    self.logger.info("[ALIGN] update: out of bounds, rotating to the left")
                    self.blackboard.wall_position = 'left'
                else:
                    self.blackboard.wall_position = None
            elif self.blackboard.out_of_the_bounds == True:
                if self.blackboard.wall_position != None: 
                    self.logger.info("[with wall] update: out of bounds, rotating to the left")
                    self.blackboard.out_of_bounds_with_wall = True
                    self.logger.info("BEORE COMMENDED FREEZE")
                    if min(leftend) > 0.7 and min(rightend) > 0.7:
                        self.blackboard.freeze_laser_scan = True
                        time.sleep(2)
                        self.blackboard.rotate_towards_nearest_wall = True
                        
                    else:
                        self.logger.info("outside THE IF ABOVE 2.5")
                        self.blackboard.freeze_laser_scan = False
                        self.blackboard.out_of_bounds_without_wall = False
            
            grouped_points = None
            all_d_m_c_endPts = []
            d_m_c_endPts = []
            line_lengths = []
            distance_collection = []
            grouped_points = len(processed_data) > 0 and online_line_detection_new(processed_data)
            if grouped_points:
                for points_on_line in grouped_points:
                    line_segment = Line(points_on_line[0], points_on_line[-1])
                    end_pts = np.array([points_on_line[0], points_on_line[-1]])
                    print("end-points: ", end_pts)
                    m, c = line_segment.equation()
                    center_wrt_laser = np.array([-0.45, 0])       # location of center of robot with respect to laser scanner
                    d = line_segment.point_dist(center_wrt_laser) # distance fom the center of the Robile
                    length = np.linalg.norm(end_pts[1] - end_pts[0])
                    line_lengths.append(length)
                    distance_collection.append(d)
                    all_d_m_c_endPts.append([d, m, c, end_pts])
                best_line = np.min(distance_collection)
                best_line_idx = distance_collection.index(best_line)
                d_m_c_endPts = all_d_m_c_endPts[best_line_idx]


            if d_m_c_endPts == []:
                self.logger.info("d_m_c_endPts is None")
                self.blackboard.out_of_the_bounds = True
                return pt.common.Status.RUNNING 
            else:
                self.logger.info("d_m_c_endPts is not None")
                self.blackboard.out_of_the_bounds = False
                
                self.blackboard.distance = d_m_c_endPts[0]
                if self.blackboard.freeze_laser_scan == False:
                    self.blackboard.slope = d_m_c_endPts[1]
                # if self.blackboard.point_at_min_distFreeze_Flag == False:
                #     self.blackboard.point_at_min_distFreeze = min(temp)
                    
                    # self.blackboard.distance = d_m_c_endPts[0]
                
                # self.blackboard.distanceForFreeze = d_m_c_endPts[0]
                self.blackboard.slopeForFreeze = d_m_c_endPts[1]
                self.blackboard.intercept = d_m_c_endPts[2]
                self.blackboard.end_pts = d_m_c_endPts[3]
                self.blackboard.out_of_the_bounds = False
                
                if self.blackboard.get('moving_along_wall')==True:
                    if point_min_idx[0] <= mid_idx:
                        self.logger.info("[ALIGN] update: out of bounds, rotating to the right")
                        self.blackboard.wall_position_out_of_bounds = 'right'
                    elif point_min_idx[0] > mid_idx:
                        self.logger.info("[ALIGN] update: out of bounds, rotating to the left")
                        self.blackboard.wall_position_out_of_bounds = 'left'
                elif self.blackboard.get('wall_position_out_of_bounds') == None:
                    self.blackboard.wall_position_out_of_bounds = self.blackboard.get('wall_position')
                    
                # else:
                #     self.blackboard.wall_position_out_of_bounds = self.blackboard.get('wall_position')
                    
                
                if self.blackboard.point_at_min_dist < self.safe_min_range:
                    self.logger.info("[LASER SCAN] update: collison warning")
                    self.blackboard.collison_warning = True
                    return pt.common.Status.RUNNING
                
                if self.blackboard.distance > 0.7:
                    if self.blackboard.slopeForFreeze > -0.04 and self.blackboard.slopeForFreeze < 0.04:
                        self.logger.info("[LASER SCAN] update: setting flags 0 under the given circumstance.")
                        self.blackboard.out_of_bounds_with_wall = False
                        self.blackboard.out_of_the_bounds = False
                        self.blackboard.back_inside_the_bounds = False
                        self.blackboard.wall_detected_warning = False
                        return pt.common.Status.RUNNING
                        
                        
                
                if self.blackboard.distance > 0.4 and self.blackboard.distance < 0.599 and self.blackboard.average_mid_point_for_corner > 0.7:
                    if self.blackboard.slopeForFreeze > -0.04 and self.blackboard.slopeForFreeze < 0.04:
                        self.logger.info("Fixing align termination")
                        # self.blackboard.freeze_laser_scan = False
                        self.blackboard.wall_detected_warning = False
                        self.blackboard.moving_along_wall = True
                        return pt.common.Status.RUNNING

                mid_array = np.array([laser_scan[mid_idx], laser_scan[mid_idx+1], laser_scan[mid_idx-1]])
                if len(mid_array) > 0:
                        self.blackboard.average_mid_point_for_corner = np.average(mid_array)
                else: 
                    self.logger.info("[LASER SCAN] update: failed: if len(mid_array) > 0")
                
                if self.blackboard.distance < 0.45  or self.blackboard.average_mid_point_for_corner < 0.65:
                    self.logger.info("[Laser Scan] update: aligning under 0.5")
                    self.blackboard.moving_forward = False
                    self.blackboard.back_inside_the_bounds = False
                    self.blackboard.distance_more_than_desired = False
                    self.blackboard.moving_forward = False
                    self.blackboard.moving_along_wall = False
                    self.blackboard.freeze_laser_scan = True
                    self.blackboard.wall_detected_warning = True
                    return pt.common.Status.RUNNING
                if self.blackboard.average_mid_point_for_corner < 0.65:
                    self.blackboard.moving_along_wall = False
                    self.blackboard.wall_detected_warning = True 
                    
                    


                if self.blackboard.distance > 0.7 or self.blackboard.back_inside_the_bounds == True:
                    
                    self.logger.info("[LASER SCAN] update: inside the resolve distance")
                    
                    mid_array = np.array([laser_scan[mid_idx], laser_scan[mid_idx+1], laser_scan[mid_idx-1]])
                    if len(mid_array) > 0:
                        self.blackboard.average_mid_point = np.average(mid_array)
                    else: 
                        self.logger.info("[LASER SCAN] update: failed: if len(mid_array) > 0")
                    
                    if np.isclose(self.blackboard.point_at_min_distFreeze, 
                                self.blackboard.average_mid_point, atol=0.2) == True:
                        self.blackboard.rotate_towards_nearest_wall = False
                        self.blackboard.moving_forward = True
                        self.blackboard.distance_more_than_desired = False
                        self.blackboard.point_at_min_distFreeze_Flag = False
                        self.blackboard.wall_detected_warning = False
                        return pt.common.Status.RUNNING
                        
                        
                    else:
                        self.logger.info("[LASER SCAN] update: inside the else resolve distance")
                        
                        self.blackboard.moving_along_wall = False
                        self.blackboard.distance_more_than_desired = True
                        self.blackboard.rotate_towards_nearest_wall = True
                        self.logger.info("[LASER SCAN] update: inside the else resolve distance")
                        
                        self.blackboard.point_at_min_distFreeze_Flag = True
                        self.blackboard.moving_forward = False
                        self.blackboard.wall_detected_warning = False
                        return pt.common.Status.RUNNING

                return pt.common.Status.RUNNING

        else:
            self.logger.info("process in the else loop")
            self.blackboard.out_of_bounds_with_wall = False
            return pt.common.Status.RUNNING 