import sys
import rclpy
import math

from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from sol_interfaces.msg import Collision

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException

from geometry_msgs.msg import Transform, Vector3

ROBOT_COLLISION_DISTANCE = 1.0

class RSManager(Node):
    def __init__(self):
        super().__init__('multi_manager')
        self.robots : list[RobotDetails] = list()
        self.declare_parameter('num_robots', 1)
        self.num_robots = self.get_parameter('num_robots').get_parameter_value().integer_value

        for i in range(self.num_robots):
            self.add_robot("/robot" + str(i + 1))

        self.collision_publisher = self.create_publisher(
            Collision,
            "/global_collisions",
            10)
        self.target_collision_publisher = self.create_publisher(
            Collision,
            "/global_target_collisions",
            10
        )
        self.timer = self.create_timer(0.1, self.control_loop)
        self.t_buffer = Buffer()
        self.t_listener = TransformListener(self.t_buffer, self)

    def control_loop(self):
        # Check 
        # This will change to 1 once tested.
        if len(self.robots) <= 1:
            return

        # Update each robots transform. (Could publish this info and just use this through system?)
        for details in self.robots:
            details.update_transform(self.t_buffer)

        # Loop through each robot combination
        for i in range(0, len(self.robots)):
            for j in range(i + 1, len(self.robots)):
                if self.robots[i].transform != None and self.robots[j].transform != None:

                    # Calculate the distance between two robots
                    dist = distance(self.robots[i].transform.translation,
                                    self.robots[j].transform.translation)
                    if dist < ROBOT_COLLISION_DISTANCE:

                        # Tell one of the robots to stop, for now robot i so the source
                        collision = Collision()
                        collision.source = self.robots[i].namespace
                        collision.target = self.robots[j].namespace
                        collision.distance = dist
                        self.collision_publisher.publish(collision)

    def add_robot(self, name : str = ""):
        '''
        Add a robot to the manager, it will allow tracking of position
        compared to others.
        '''
        if name in self.robots:
            return
        self.robots.append(RobotDetails(name))


class RobotDetails():
    def __init__(self, name):
        self.namespace = name
        self.transform : Transform = None
        self.target = None
    
    def update_transform(self, buffer : Buffer):
        '''
        Updates the transform of a given robot, this is to
        track its position against others.
        '''
        ns = self.namespace[1:]
        transform : Transform = None
        try:
            transform = buffer.lookup_transform(
                ns + '/map',
                ns + '/base_footprint',
                rclpy.time.Time()).transform
        except TransformException as e:
            print(e)
        self.transform = transform

def distance(v1 : Vector3, v2 : Vector3):
    '''
    Calculate the distance difference between two vectors.
    :param v1: the first vector.
    :param v2: the second vector.
    '''
    diffx = v1.x - v2.x
    diffy = v1.y - v2.y
    return math.sqrt(diffx * diffx + diffy * diffy)

def main(args=None):
    rclpy.init(args = args)

    rsm = RSManager()

    try:
        rclpy.spin(rsm)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rsm.destroy_node()
        rclpy.try_shutdown()
