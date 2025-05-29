import sys
import math
import rclpy

from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from nav2_simple_commander.robot_navigator import BasicNavigator

from geometry_msgs.msg import PoseStamped, Twist, Transform, Point, Quaternion
from assessment_interfaces.msg import ItemHolders
from std_msgs.msg import String

from sol_interfaces.msg import Target, Collision
from enum import Enum

LOCK_ON_DISTANCE = 1.0
LOCK_ON_DEVIATION = 0.5
DISTANCE_DEVIATION = 0.5
MAJOR_TURNING_ANGLE = 0.15
MINOR_TURNING_ANGLE = 0.05

FAST_LINEAR_SPEED = 0.26
FAST_TURNING_SPEED = 0.5
FINE_TURNING_SPEED = 0.1

SWAP_MARGIN = 0.75

CONTROL_LOOP_INTERVAL = 0.1
NO_OVERRIDE_TIME = 0.5

class State(Enum):
    SET_GOAL = 0
    NAVIGATING = 1
    LOCKED = 2
    SET_RETURN = 3
    RETURNING = 4
    LOST = 6
    TEST = 7

class NavigatorNode(Node):
    def __init__(self):
        super().__init__('navigation_manager')

        # Declare arguments
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)

        self.create_timer(CONTROL_LOOP_INTERVAL, self.control_loop)
        self.target : Target = None

        # Subscribers
        self.navigation_subscriber = self.create_subscription(
            Target,
            "target",
            self.navigation_callback,
            10)
        self.state = State.SET_GOAL
        self.item_target : Target = None
        self.current_lockon = 0
        self.target_found = False
        self.holder_subscriber = self.create_subscription(
            ItemHolders,
            "/item_holders",
            self.holder_callback,
            10)
        self.holding : bool = False
        self.holding_value : int = 0
        self.override_subscriber = self.create_subscription(
            Collision,
            "override",
            self.override_callback,
            10)
        self.last_override = 0.5
        self.override = False

        # Publishers
        self.cmd_publisher = self.create_publisher(
            Twist,
            "cmd_vel",
            10)
        self.state_publisher = self.create_publisher(
            String,
            "state",
            10)
        self.swapping = False
        self.done_swap = False

        # Robot Transform
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialise Robot Navigator
        self.navigator = BasicNavigator(namespace=self.get_namespace())
        init_pose = PoseStamped()
        init_pose.header.frame_id = "map"
        init_pose.pose.orientation.w = 1.0
        init_pose.pose.position.x = self.get_parameter('x').get_parameter_value().double_value
        init_pose.pose.position.y = self.get_parameter('y').get_parameter_value().double_value        
        self.navigator.setInitialPose(init_pose)
        self.navigator.waitUntilNav2Active()
    
    def navigation_callback(self, msg : Target):
        '''
        Navigation Callback which stores the current best found target data.
        :param msg: target data.
        '''
        self.item_target = msg
        self.target_found = True

    def holder_callback(self, msg : ItemHolders):
        '''
        Callback when the item_holder data is received, this is to check
        whether this robot is holding an item.
        :param msg: item holder data.
        '''
        for holder in msg.data:
            if holder.robot_id == self.get_namespace()[1:]:
                self.holding = holder.holding_item
                self.holding_value = holder.item_value
                return

    def override_callback(self, msg : Collision):
        '''
        A callback to stop the robot and let the proximity detector
        override it's moving functionality (e.g for evading).
        :param msg: collision data (unused).
        '''
        if self.last_override > NO_OVERRIDE_TIME:
            if self.state is State.NAVIGATING:
                self.state = State.SET_GOAL
                self.target = None
            elif self.state is State.RETURNING:
                self.state = State.SET_RETURN
        self.last_override = 0.0

    def create_pose(self, position : Point, orientation : Quaternion = None):
        '''
        Creates a pose to be used within the navigator in which
        its frame id is the global map.
        :param position: the positional data.
        :param orientation: the orientational data.
        '''
        if orientation == None:
            orientation = Quaternion()
            orientation.w = 1.0
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()     
        pose.pose.position = position
        pose.pose.orientation = orientation
        return pose

    def control_loop(self):
        self.last_override += CONTROL_LOOP_INTERVAL

        # Read the transformation buffer to get the latest transformation
        ns = self.get_namespace()[1:]
        transform : Transform = None
        try:
            transform = self.tf_buffer.lookup_transform(
                ns + '/map',
                ns + '/base_footprint',
                rclpy.time.Time()).transform
        except Exception as e:
            print(e)

        if transform == None:
            return

        # Check if an override has occurred and cancel the ongoing task.
        if self.last_override < NO_OVERRIDE_TIME:
            if not self.override:
                self.navigator.cancelTask()
                self.override = True
            return
        else:
            self.override = False

        # Check if a valid target is visible
        if self.target == None and not self.holding and not self.state == State.LOST: 
            if self.state is State.SET_GOAL:
                self.target = self.item_target
            if self.item_target == None:
                self.state = State.LOST
            if self.state is not State.RETURNING:
                return

        match self.state:
            case State.SET_GOAL:
                # This can be removed later
                if self.holding:
                    self.state = State.SET_RETURN
                    return

                # Set the next navigation goal
                self.get_logger().info("SET, NOW NAVIGATING!")
                self.navigator.goThroughPoses([self.create_pose(self.target.position)])
                self.state = State.NAVIGATING
            case State.NAVIGATING:
                
                # Get the feeback of the navigation to check if the robot is stuck
                feedback = self.navigator.getFeedback()
                if feedback is not None and feedback.estimated_time_remaining.nanosec == 0:
                    self.state = State.SET_GOAL
                    self.get_logger().info("ROUTE NOT SET, REPLOTTING!") 
                    return

                # Calculate the distance deviation
                self.target.distance = calculateDistance(
                    self.target.position.x, 
                    self.target.position.y,
                    transform.translation.x,
                    transform.translation.y)
                distanceDiff = abs(self.target.distance - self.item_target.distance)

                # Checks if the target is of better value or deviated too much and update nav goal.
                if self.item_target.value > self.target.value or (distanceDiff > DISTANCE_DEVIATION and self.item_target.value >= self.target.value):
                    self.get_logger().info("NAV GOAL UPDATED!")
                    self.target = self.item_target
                    self.navigator.goThroughPoses([self.create_pose(self.target.position)])
                elif self.target.distance < LOCK_ON_DISTANCE and self.target_found:
                    self.get_logger().info("NAVIGATED, NOW LOCKED!")
                    self.navigator.cancelTask()
                    self.current_lockon = self.item_target.distance
                    self.state = State.LOCKED
                # elif self.holding and self.holding_value >= self.item_target.value:
                #     self.get_logger().info("ITEM NAVIGATED TO, NOW RETURNING!")
                #     self.state = State.SET_RETURN
                elif self.holding:
                    self.done_swap = True
                
            case State.LOCKED:
                linear = 0.0
                angular = 0.0
                
                # Compare the item target angle to orient the robot
                if abs(self.item_target.angle) > MAJOR_TURNING_ANGLE:
                    angular = FAST_TURNING_SPEED * norm(self.item_target.angle)
                elif abs(self.item_target.angle) > MINOR_TURNING_ANGLE:
                    angular = FINE_TURNING_SPEED * norm(self.item_target.angle)

                # Check if the correct item is held and if the robot has performed a swap on the way
                if self.holding and self.holding_value >= self.target.value:
                    self.state = State.SET_RETURN
                    linear = 0.0
                    angular = 0.0
                elif self.holding:
                    self.done_swap = True
                    linear = FAST_LINEAR_SPEED
                else:
                    linear = FAST_LINEAR_SPEED

                # When the target is too close this can't apply
                if self.item_target.distance > LOCK_ON_DISTANCE + LOCK_ON_DEVIATION:
                    self.get_logger().info("TARGET LOST, FINDING ANOTHER!")
                    self.state = State.LOST
                    linear = 0.0
                    angular = 0.0

                # Publish the lock on data
                twist = Twist()
                twist.linear.x = linear
                twist.angular.z = angular
                self.cmd_publisher.publish(twist)

            case State.SET_RETURN:
                
                # Sets a return route
                self.get_logger().info("ITEM COLLETED, RETURNING!")
                home_zone = Point()
                home_zone.x = -3.5
                home_zone.y = transform.translation.y
                self.navigator.goThroughPoses([self.create_pose(home_zone)])
                self.state = State.RETURNING
            case State.RETURNING:
                    
                # Get the feeback of the navigation to check if the robot is stuck
                feedback = self.navigator.getFeedback()
                if feedback is not None and feedback.estimated_time_remaining.nanosec == 0:
                    self.state = State.SET_RETURN
                    self.get_logger().info("ROUTE NOT SET, REPLOTTING!") 
                    return

                # When a lower item value is a fair bit south of the robot you can swap
                if self.item_target.value < self.holding_value and self.item_target.position.x + SWAP_MARGIN < transform.translation.x:
                    if not self.swapping and not self.done_swap:
                        
                        # Initiate a swap
                        i_rotation = Quaternion()
                        i_rotation.y = 1.0
                        item_pose = self.create_pose(self.item_target.position, i_rotation)
                        home_point = Point()
                        home_point.x = -3.5
                        home_point.y = item_pose.pose.position.y
                        home_pose = self.create_pose(home_point)
                        self.get_logger().info("ATTEMPTING SWAP!")
                        if self.navigator.goThroughPoses([item_pose, home_pose]):
                            self.swapping = True

                # If the item is held then return g
                if not self.holding:
                    self.get_logger().info("RETURNED, SPINNING!")
                    self.target = None
                    self.target_found = False
                    self.swapping = False
                    self.done_swap = False
                    self.state = State.SET_GOAL
            # Need to find more ways to get into this state, 
            # Maybe a timer system if it hasn't found a target in x time
            case State.LOST:
                
                # Tell the proximity detector the robot is lost
                if self.item_target != None:
                    self.state = State.SET_GOAL
            
        state = String()
        state.data = str(self.state.name)
        self.state_publisher.publish(state)


def norm(value : float):
    '''
    Calculate the directionl norm of a float value.
    :param value: float value.
    '''
    if value < 0:
        return -1
    if value > 0:
        return 1
    if value == 0:
        return 0

def calculateDistance(x1, y1, x2, y2):
    '''
    Calculate the distance between two 2D points.
    :param x1: first x value.
    :param y1: first y value.
    :param x2: second x value.
    :param y2: second y value.
    '''
    diffx = x1 - x2
    diffy = y1 - y2
    return math.sqrt(diffx * diffx + diffy * diffy)

def main(args=None):
    rclpy.init(args = args)

    nn = NavigatorNode()

    try:
        rclpy.spin(nn)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        nn.destroy_node()
        rclpy.try_shutdown()