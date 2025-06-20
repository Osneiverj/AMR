import rclpy
from rclpy.lifecycle import LifecycleNode

from std_srvs.srv import Empty
from nav2_msgs.srv import LoadMap, ManageLifecycleNodes
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
import os


class Orchestrator(LifecycleNode):
    def __init__(self):
        super().__init__("ui_orchestrator")

        # Servicios expuestos a la UI
        self.create_service(Empty, "ui/start_slam", self.start_slam)
        self.create_service(Empty, "ui/stop_slam", self.stop_slam)
        self.create_service(LoadMap, "ui/load_map", self.load_map)
        self.create_service(Empty, "ui/start_nav2", self.start_nav2)
        self.create_service(Empty, "ui/stop_nav2", self.stop_nav2)

        # Lifecycle clients for SLAM Toolbox and Map Server
        self.slam_lifecycle_client = self.create_client(
            ChangeState, "/lifecycle_manager_slam_toolbox/change_state"
        )

        # Cliente para map_server y su servicio load_map
        self.map_load_client = self.create_client(
            LoadMap, "/map_server/load_map"
        )

        # Lifecycle managers
        self.map_lifecycle_client = self.create_client(
            ChangeState, "/lifecycle_manager_map_server/change_state"
        )
        self.nav2_lifecycle_client = self.create_client(
            ManageLifecycleNodes, "/lifecycle_manager_navigation/manage_nodes"
        )

    # ---------------------- Helper methods ----------------------
    def _call_change_state(self, client, name: str, transition: int) -> bool:
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"Service {name} unavailable")
            return False
        req = ChangeState.Request()
        req.transition.id = transition
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        if res and res.success:
            return True
        self.get_logger().error(
            f"Transition {transition} on {name} failed"
        )
        return False

    def _call_manage(self, client, name: str, command: int) -> bool:
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"Lifecycle service {name} unavailable")
            return False
        req = ManageLifecycleNodes.Request(command=command)
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        if res and res.success:
            return True
        self.get_logger().error(
            f"Lifecycle command {command} on {name} failed"
        )
        return False

    # ---------------------- SLAM / Nav2 control ----------------------
    def start_slam(self, request, response):
        # Ensure nav2 stack is down before mapping
        self.get_logger().info("Stopping Nav2 stack for mapping")
        self._call_manage(
            self.nav2_lifecycle_client,
            "/lifecycle_manager_navigation/manage_nodes",
            ManageLifecycleNodes.Request.SHUTDOWN,
        )
        self.get_logger().info("Cleaning up Map Server")
        self._call_change_state(
            self.map_lifecycle_client,
            "/lifecycle_manager_map_server/change_state",
            Transition.TRANSITION_CLEANUP,
        )
        self.get_logger().info("Configuring SLAM Toolbox")
        self._call_change_state(
            self.slam_lifecycle_client,
            "/lifecycle_manager_slam_toolbox/change_state",
            Transition.TRANSITION_CONFIGURE,
        )
        self.get_logger().info("Activating SLAM Toolbox")
        self._call_change_state(
            self.slam_lifecycle_client,
            "/lifecycle_manager_slam_toolbox/change_state",
            Transition.TRANSITION_ACTIVATE,
        )
        return response

    def stop_slam(self, request, response):
        self.get_logger().info("Deactivating SLAM Toolbox")
        self._call_change_state(
            self.slam_lifecycle_client,
            "/lifecycle_manager_slam_toolbox/change_state",
            Transition.TRANSITION_DEACTIVATE,
        )
        return response

    def load_map(self, request, response):
        # Switch from mapping to navigation: stop SLAM, reset map server, load map, start Nav2
        self.get_logger().info("Deactivating SLAM Toolbox before loading map")
        self._call_change_state(
            self.slam_lifecycle_client,
            "/lifecycle_manager_slam_toolbox/change_state",
            Transition.TRANSITION_DEACTIVATE,
        )

        # Reset map server to ensure fresh configuration
        self.get_logger().info("Cleaning up Map Server")
        self._call_change_state(
            self.map_lifecycle_client,
            "/lifecycle_manager_map_server/change_state",
            Transition.TRANSITION_CLEANUP,
        )

        # Set map YAML file for next startup
        os.system(
            f"ros2 param set /map_server yaml_filename {request.map_url}"
        )

        self.get_logger().info("Configuring Map Server")
        self._call_change_state(
            self.map_lifecycle_client,
            "/lifecycle_manager_map_server/change_state",
            Transition.TRANSITION_CONFIGURE,
        )
        self._call_change_state(
            self.map_lifecycle_client,
            "/lifecycle_manager_map_server/change_state",
            Transition.TRANSITION_ACTIVATE,
        )

        self.get_logger().info(f"Loading map via service: {request.map_url}")
        if not self.map_load_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service /map_server/load_map unavailable")
            response.result = LoadMap.Response.RESULT_FAILURE
            return response
        fut = self.map_load_client.call_async(request)
        rclpy.spin_until_future_complete(self, fut)
        result = fut.result()
        if result is None or result.result != LoadMap.Response.RESULT_SUCCESS:
            self.get_logger().error("LoadMap call failed")
            response.result = LoadMap.Response.RESULT_FAILURE
            return response

        self.get_logger().info("Starting Nav2 stack after map load")
        self._call_manage(
            self.nav2_lifecycle_client,
            "/lifecycle_manager_navigation/manage_nodes",
            ManageLifecycleNodes.Request.STARTUP,
        )
        return result

    def start_nav2(self, request, response):
        self.get_logger().info("Starting Nav2 stack")
        self._call_manage(
            self.nav2_lifecycle_client,
            "/lifecycle_manager_navigation/manage_nodes",
            ManageLifecycleNodes.Request.STARTUP,
        )
        return response

    def stop_nav2(self, request, response):
        self.get_logger().info("Shutting down Nav2 stack")
        self._call_manage(
            self.nav2_lifecycle_client,
            "/lifecycle_manager_navigation/manage_nodes",
            ManageLifecycleNodes.Request.SHUTDOWN,
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = Orchestrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
