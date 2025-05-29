import sys
import rclpy
import math

from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException

from assessment_interfaces.msg import ItemList
from sol_interfaces.msg import Target, TargetArray

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Transform, Quaternion, Vector3, Point
from std_msgs.msg import ColorRGBA

# Work of D. V. Lu, et al. https://github.com/DLu/tf_transformations.
from tf_transformations import euler_from_quaternion


ITEM_RADIUS = 75
CAMERA_ZOOM = 1
CENTER_TO_CAMERA = 0.076
F_VALUE = 650

class FinderNode(Node):
    def __init__(self):
        super().__init__('target_finder')
        self.visual = True
        self.item_subscriber = self.create_subscription(
            ItemList,
            "items",
            self.item_callback,
            10)
        self.timer = self.create_timer(0.5, self.control_loop)
        self.items : ItemList = ItemList()
        self.target_publisher = self.create_publisher(
            TargetArray,
            "targets",
            10)
        if self.visual:
            self.marker_publisher = self.create_publisher(
                Marker,
                "targets/visual",
                10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def item_callback(self, items : ItemList):
        '''
        Callback to handle item data.
        '''
        self.items = items

    def control_loop(self):
        # Return if no items are present
        if len(self.items.data) == 0:
            self.target_publisher.publish(TargetArray())
            return

        # Reading the transformation buffer to get the latest transformation
        ns = self.get_namespace()[1:]
        transform : Transform = None
        try:
            transform = self.tf_buffer.lookup_transform(
                ns + '/map',
                ns + '/base_footprint',
                rclpy.time.Time()).transform
        except TransformException as e:
            print(e)

        if transform == None:
            return
        
        target_array = TargetArray()

        # Loops through all targets found by the camera
        for item in self.items.data:

            # Filter out items held by other robots
            if item.y > 10:
                continue

            # Returns the estimated distance and position of the item
            item_distance = estimate_item_distance(item.diameter, item.x)
            position = estimate_item_position(item.x, item_distance, transform)

            # Creates a target object for the item
            target = Target()
            target.position.x = clamp(position.x, -4.0, 3.0)
            target.position.y = clamp(position.y, -3.0, 3.0)
            target.angle = position.w
            target.distance = item_distance
            target.value = item.value

            # Add to output array
            target_array.targets.append(target)

        # Publish target data
        self.target_publisher.publish(target_array)

        if not self.visual:
            return
    
        # Publish visual data
        marker = draw_points(target_array)
        marker.id = 0
        marker.header.frame_id = "/map"
        marker.header.stamp = self.get_clock().now().to_msg()
        self.marker_publisher.publish(marker)

def estimate_item_distance(diameter : float, xangle : float):
    '''
    Estimate item distance based on camera coordinates.
    :param diameter: the diameter of the sphere.
    :param xangle: the x camera coordinate.
    '''
    return ITEM_RADIUS / diameter * CAMERA_ZOOM * (1 + abs(xangle) / 1000)

def estimate_item_position(xpixels : float, distance : float, robotpose : Transform) -> Quaternion:
    '''
    Estimate the position based on distance and camera x,
    using perspective projection. Relative to the robots pose.
    :param xpixels: the x camera coordinate.
    :param distance: distance between the object and camera.
    :param robotpose: the pose of the robot.
    '''

    # f = 650 
    xangle = math.atan(xpixels / F_VALUE)
    (pitch, roll, yaw) = euler_from_quaternion([robotpose.rotation.x,
                                                robotpose.rotation.y,
                                                robotpose.rotation.z,
                                                robotpose.rotation.w])
    addX = CENTER_TO_CAMERA * math.cos(yaw)
    addy = CENTER_TO_CAMERA * math.sin(yaw)
    itemAngle = yaw + xangle
    x = robotpose.translation.x + distance * math.cos(itemAngle) + addX
    y = robotpose.translation.y + distance * math.sin(itemAngle) + addy
    result = Quaternion()
    result.x = x
    result.y = y
    result.z = 0.0
    result.w = xangle
    return result


def clamp(value : float, min : float, max : float): 
    '''
    Clamp a value between a maximum and minimum.
    '''
    if value < min: 
        return min
    elif value > max: 
        return max
    else: 
        return value 

def main(args=None):
    rclpy.init(args = args)

    fn = FinderNode()

    try:
        rclpy.spin(fn)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        fn.destroy_node()
        rclpy.try_shutdown()



def draw_points(target_array : TargetArray):
    '''
    Relay all potential targets as spheres to show on rviz.
    Looks similar to the items within gazebo.
    '''
    pointScale = Vector3()
    pointScale.x = 0.2
    pointScale.y = 0.2
    pointScale.z = 0.2

    color = ColorRGBA()
    color.r = 0.0
    color.g = 1.0
    color.b = 0.0
    color.a = 1.0

    marker = Marker()
    marker.type = 7 # Sphere list looked the best for visualisation
    marker.color = color
    marker.scale = pointScale

    points = []
    for target in target_array.targets:
        point = Point()
        point.x = target.position.x
        point.y = target.position.y
        point.z = 0.1
        points.append(point)

    marker.points = points
    return marker
