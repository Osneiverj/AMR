import rclpy
import py_trees
import py_trees_ros
import json
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

class NavigateToPoseBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name, node, pose):
        super(NavigateToPoseBehavior, self).__init__(name)
        self.node = node
        self.pose = pose
        self.action_client = py_trees_ros.actions.ActionClient(
            name="NavigateToPose",
            action_spec=NavigateToPose,
            action_goal=self.create_goal(),
            node_name=self.node.get_name(),
            wait_for_server_timeout_sec=5.0
        )

    def create_goal(self):
        goal = NavigateToPose.Goal()
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = self.pose['x']
        goal.pose.pose.position.y = self.pose['y']
        goal.pose.pose.orientation.w = self.pose['w']
        return goal

    def setup(self, **kwargs):
        self.node.get_logger().info(f"[{self.name}] setting up action client")
        self.action_client.setup(**kwargs)

    def update(self):
        self.node.get_logger().info(f"[{self.name}] ticking")
        return self.action_client.tick()

def create_mission_tree(node, mission_definition):
    mission_json = json.loads(mission_definition)
    root = py_trees.composites.Sequence(name="MissionSequence")
    
    for task in mission_json['tasks']:
        if task['type'] == 'navigate_to_pose':
            behavior = NavigateToPoseBehavior(
                name=f"NavigateTo_{task['details']['name']}",
                node=node,
                pose=task['details']['pose']
            )
            root.add_child(behavior)
            
    return py_trees.trees.BehaviourTree(root)
