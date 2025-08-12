import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer

from orchestrator_interfaces.action import ExecuteMission
from orchestrator.behaviors import create_mission_tree
from orchestrator.utils import call_change_state, call_manage_lifecycle_nodes

import py_trees
import py_trees_ros

class Orchestrator(LifecycleNode):
    def __init__(self):
        super().__init__(
            "ui_orchestrator",
            automatically_declare_parameters_from_overrides=True,
        )
        self.cb_group = ReentrantCallbackGroup()
        self.mission_action_server = ActionServer(
            self,
            ExecuteMission,
            'execute_mission',
            self.execute_mission_callback,
            callback_group=self.cb_group
        )
        self.bt_runner = None

    def execute_mission_callback(self, goal_handle):
        self.get_logger().info('Executing mission...')
        mission_definition = goal_handle.request.mission_definition
        
        try:
            self.bt_runner = create_mission_tree(self, mission_definition)
            self.bt_runner.setup(timeout=15.0)
            
            # Tick the tree until it finishes
            while self.bt_runner.root.status in [py_trees.common.Status.RUNNING, py_trees.common.Status.INVALID]:
                self.bt_runner.tick()
                rclpy.spin_once(self, timeout_sec=0.1)
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Mission cancelled.')
                    self.bt_runner.shutdown()
                    goal_handle.canceled()
                    return ExecuteMission.Result(success=False, message="Mission cancelled by user.")

            goal_handle.succeed()
            result = ExecuteMission.Result()
            result.success = True
            result.message = "Mission completed successfully!"
            return result

        except Exception as e:
            self.get_logger().error(f'Failed to execute mission: {e}')
            goal_handle.abort()
            return ExecuteMission.Result(success=False, message=f"Mission failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = Orchestrator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
