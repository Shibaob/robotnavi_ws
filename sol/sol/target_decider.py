import sys
import rclpy

from sol_interfaces.msg import TargetArray, Target

from std_msgs.msg import ColorRGBA, String
from geometry_msgs.msg import Quaternion, Vector3, Point, Pose
from visualization_msgs.msg import Marker

from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

class TargetDecider(Node):
    def __init__(self):
        super().__init__('target_decider')
        self.visual = True
        self.switching = False
        self.targets_subscriber = self.create_subscription(
            TargetArray,
            "targets",
            self.target_callback,
            10)
        self.targets : TargetArray = None
        
        self.state_subscriber = self.create_subscription(
            String,
            "state",
            self.state_callback,
            10)
        self.robot_state = ""

        self.target_publisher = self.create_publisher(
            Target,
            "target",
            10)
        self.visual_publisher = self.create_publisher(
            Marker,
            "target/visual",
            10)
        self.timer = self.create_timer(0.25, self.control_loop)

    def target_callback(self, msg):
        '''
        Gets all the targets to determine the best one.
        :param msg: targets data.
        '''
        self.targets = msg

    def state_callback(self, msg):
        '''
        Callback to store the robots state to determine 
        which targeting system to use.
        :param msg: state data.
        '''
        self.robot_state = msg.data

    def control_loop(self):
        if self.targets is None or len(self.targets.targets) == 0:
            return
        
        # Calculate the best possible target to travel to
        best_score = 0.0
        best_target : Target = None
        for index, target in enumerate(self.targets.targets):

            # If switching then invert the score 
            target_score = self.calculate_item_score(target.distance, target.value)

            # Determine if this target has the best score
            if index == 0 or target_score < best_score:
                best_score = target_score
                best_target = target
        
        # Publish target
        self.target_publisher.publish(best_target)

        if not self.visual:
            return
        
        # Publish visual
        marker = draw_arrow(best_target.position.x, best_target.position.y)
        marker.id = 0
        marker.header.frame_id = "/map"
        marker.header.stamp = self.get_clock().now().to_msg()
        self.visual_publisher.publish(marker)
        

    def calculate_item_score(self, distance : float, value : float):
        '''
        Calculates the item score and determines to go for
        the most valuable or least valuable (when switching).
        :param distance: the distance to target.
        :param value: the target's value
        '''
        if self.robot_state == 'RETURNING':
            return distance * value
        else:
            return distance * (1 / (value * value))

def main(args=None):
    rclpy.init(args = args)

    td = TargetDecider()

    try:
        rclpy.spin(td)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        td.destroy_node()
        rclpy.try_shutdown()

def draw_arrow(x, y) -> Marker:
    '''
    Draws an arrow to display on rviz for debugging and visual aid.
    :param x: x coorindate of arrow
    :param y: y coorindate of arrow
    '''
    arrowScale = Vector3()
    arrowScale.x = 0.5
    arrowScale.y = 0.1
    arrowScale.z = 0.1

    arrowRotation = Quaternion()
    arrowRotation.w = 0.71
    arrowRotation.x = 0.0
    arrowRotation.y = 0.71
    arrowRotation.z = 0.0

    arrowPosition = Point()
    arrowPosition.x = float(x)
    arrowPosition.y = float(y)
    arrowPosition.z = 0.75

    color = ColorRGBA()
    color.r = 1.0
    color.g = 0.0
    color.b = 0.0
    color.a = 1.0

    pose = Pose()
    pose.position = arrowPosition
    pose.orientation = arrowRotation

    marker = Marker()
    marker.type = 0
    marker.color = color
    marker.pose = pose
    marker.scale = arrowScale
    return marker