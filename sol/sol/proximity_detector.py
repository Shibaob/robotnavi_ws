import sys
import math
import rclpy

from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from sol_interfaces.msg import Collision

TURN_DISTANCE = 0.65
COLLISION_DISTANCE = 0.65
EVADE_DISTANCE = 0.80
STOP_DISTANCE = 0.50
STOP_BUFFER = 0.05
TURN_ANGLE = 60
LINEAR_SPEED = 0.3
ANGULAR_SPEED = 1.5

CLOSE_ANGLE = 1.5
NO_COLLISION_TIMER = 0.5

class ProximityDetector(Node):
    def __init__(self):
        super().__init__('proximity_detector')
        self.collision_subscriber = self.create_subscription(
            Collision,
            "/global_collisions",
            self.collision_callback,
            10)
        self.collision_stop = False

        self.scanner_subscriber = self.create_subscription(
            LaserScan,
            "scan",
            self.scanner_callback,
            10)
        self.lost_subscriber = self.create_subscription(
            String,
            "state",
            self.lost_callback,
            10)
        self.no_target = False
        self.ready = False
        
        self.cmd_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10)
        self.override_publisher = self.create_publisher(
            Collision,
            "override",
            10)
        self.manual_drive = False
        self.timer = self.create_timer(0.1, self.control_loop)

        self.turn = False
        self.left = False
        self.stop = False

    def collision_callback(self, msg : Collision):
        '''
        Checks if the robot needs to perform evasive measures 
        when involved within a collision.
        :param msg: potential collision information.
        '''
        if msg.target == self.get_namespace():
            if msg.distance < COLLISION_DISTANCE:
                self.collision_stop = True
            else:
                self.collision_stop = False
        
    def scanner_callback(self, msg : LaserScan):
        '''
        Reads to robot's LIDAR information to ensure the robot
        is not too close to an obstacle.
        :param msg: the LIDAR information.
        '''
        minRay = 0
        left = False
        for i in range(TURN_ANGLE):
            if (msg.ranges[i] < msg.ranges[minRay]):
                minRay = i

        for i in range(360 - TURN_ANGLE, 360):
            if (msg.ranges[i] < msg.ranges[minRay]):
                left = True
                minRay = i

        if msg.ranges[minRay] >= STOP_DISTANCE + STOP_BUFFER:
            self.stop = False

        if msg.ranges[minRay] >= TURN_DISTANCE:
            self.turn = False
            return

        if msg.ranges[minRay] < STOP_DISTANCE:
            self.stop = True

        if not left:
            # Turn Right
            self.turn = True
            self.left = False
            return

        if left:
            # Turn Left
            self.turn = True
            self.left = True
            return

        self.turn = False

    def lost_callback(self, msg):
        '''
        Checks if the robot's state is lost.
        This will allow the robot to randomly navigate
        until a target is found.
        '''
        if not self.ready:
            self.ready = True

        if msg.data == "LOST":
            self.no_target = True
            self.manual_drive = True
        else:
            self.no_target = False

    def control_loop(self):

        # Prevents the robot randomly turning when setting the map.
        if not self.ready:
            return

        # Checks if the robot is involved with a collision to override the
        # navigation controller
        if not self.no_target or self.collision_stop:
            if self.stop or self.collision_stop:
                self.manual_drive = True
                collision = Collision()
                collision.source = self.get_namespace()
                collision.target = "Object"
                self.override_publisher.publish(collision)
            else:
                if self.manual_drive is True:
                    self.pub_cmd_vel(0, 0)
                self.manual_drive = False

        # Activates manual drive mode to avoid close obstacles.
        if self.manual_drive:
            l = 0
            a = 0
            if not self.stop:
                l = LINEAR_SPEED

            if self.turn and self.left:
                a = ANGULAR_SPEED
            elif self.turn:
                a = -ANGULAR_SPEED

            self.pub_cmd_vel(l, a)

    def pub_cmd_vel(self, l: float, a: float):
        '''
        Publishes a cmd_vel message to move the robot
        in a given direction.
        :param l: linear information.
        :param a: angular information.
        '''
        msg = Twist()
        msg.linear.x = float(l)
        msg.angular.z = float(a)
        self.cmd_publisher.publish(msg)

def main(args=None):
    rclpy.init(args = args)

    pd = ProximityDetector()

    try:
        rclpy.spin(pd)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        pd.destroy_node()
        rclpy.try_shutdown()