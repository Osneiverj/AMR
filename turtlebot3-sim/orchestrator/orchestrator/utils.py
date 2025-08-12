import rclpy
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State
from nav2_msgs.srv import ManageLifecycleNodes

def call_change_state(node, client, name: str, transition: int) -> bool:
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error(f"Service {name} unavailable")
        return False
    req = ChangeState.Request()
    req.transition.id = transition
    fut = client.call_async(req)
    rclpy.spin_until_future_complete(node, fut)
    res = fut.result()
    if res and res.success:
        return True
    node.get_logger().error(f"Transition {transition} on {name} failed")
    return False

def call_manage_lifecycle_nodes(node, client, command: int) -> bool:
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error(f"Lifecycle service unavailable")
        return False
    req = ManageLifecycleNodes.Request(command=command)
    fut = client.call_async(req)
    rclpy.spin_until_future_complete(node, fut)
    res = fut.result()
    if res and res.success:
        return True
    node.get_logger().error(f"Lifecycle command {command} failed")
    return False
