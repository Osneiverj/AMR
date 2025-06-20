import rclpy
from rclpy.lifecycle import LifecycleNode

from std_srvs.srv import Empty
from nav2_msgs.srv import LoadMap, ManageLifecycleNodes
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State


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
            ChangeState, "/slam_toolbox/change_state"
        )
        self.slam_state_client = self.create_client(
            GetState, "/slam_toolbox/get_state"
        )

        # Cliente para map_server y su servicio load_map
        self.map_load_client = self.create_client(
            LoadMap, "/map_server/load_map"
        )
        self.map_lifecycle_client = self.create_client(
            ChangeState, "/map_server/change_state"
        )
        self.map_state_client = self.create_client(
            GetState, "/map_server/get_state"
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

    def _get_state(self, client, name: str):
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"Service {name} unavailable")
            return None
        req = GetState.Request()
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        if res is None:
            self.get_logger().error(f"Failed to get state from {name}")
            return None
        return res.current_state.id

    # ---------------------- SLAM / Nav2 control ----------------------
    def start_slam(self, request, response):
        # Ensure nav2 stack is down before mapping
        self.get_logger().info("Stopping Nav2 stack for mapping")
        self._call_manage(
            self.nav2_lifecycle_client,
            "/lifecycle_manager_navigation/manage_nodes",
            ManageLifecycleNodes.Request.SHUTDOWN,
        )

        # Limpia map_server solo si estaba activo
        map_state = self._get_state(self.map_state_client, "/map_server/get_state")
        if map_state == State.PRIMARY_STATE_ACTIVE:
            self._call_change_state(
                self.map_lifecycle_client,
                "/map_server/change_state",
                Transition.TRANSITION_DEACTIVATE,
            )
            self._call_change_state(
                self.map_lifecycle_client,
                "/map_server/change_state",
                Transition.TRANSITION_CLEANUP,
            )

        # Configura y activa SLAM Toolbox solo si es necesario
        slam_state = self._get_state(self.slam_state_client, "/slam_toolbox/get_state")
        if slam_state == State.PRIMARY_STATE_UNCONFIGURED:
            self._call_change_state(
                self.slam_lifecycle_client,
                "/slam_toolbox/change_state",
                Transition.TRANSITION_CONFIGURE,
            )
            slam_state = State.PRIMARY_STATE_INACTIVE
        if slam_state == State.PRIMARY_STATE_INACTIVE:
            self._call_change_state(
                self.slam_lifecycle_client,
                "/slam_toolbox/change_state",
                Transition.TRANSITION_ACTIVATE,
            )
        return response

    def stop_slam(self, request, response):
        self.get_logger().info("Deactivating SLAM Toolbox")
        slam_state = self._get_state(self.slam_state_client, "/slam_toolbox/get_state")
        if slam_state == State.PRIMARY_STATE_ACTIVE:
            self._call_change_state(
                self.slam_lifecycle_client,
                "/slam_toolbox/change_state",
                Transition.TRANSITION_DEACTIVATE,
            )
        return response

    def load_map(self, request, response):
        # Switch from mapping to navigation: stop SLAM, reset map server, load map, start Nav2
        self.get_logger().info("Deactivating SLAM Toolbox before loading map")
        slam_state = self._get_state(self.slam_state_client, "/slam_toolbox/get_state")
        if slam_state == State.PRIMARY_STATE_ACTIVE:
            self._call_change_state(
                self.slam_lifecycle_client,
                "/slam_toolbox/change_state",
                Transition.TRANSITION_DEACTIVATE,
            )

        # Limpia map_server solo si estaba activo
        map_state = self._get_state(self.map_state_client, "/map_server/get_state")
        if map_state == State.PRIMARY_STATE_ACTIVE:
            self._call_change_state(
                self.map_lifecycle_client,
                "/map_server/change_state",
                Transition.TRANSITION_DEACTIVATE,
            )
            self._call_change_state(
                self.map_lifecycle_client,
                "/map_server/change_state",
                Transition.TRANSITION_CLEANUP,
            )

        # Configura map_server si es necesario
        map_state = self._get_state(self.map_state_client, "/map_server/get_state")
        if map_state == State.PRIMARY_STATE_UNCONFIGURED:
            self._call_change_state(
                self.map_lifecycle_client,
                "/map_server/change_state",
                Transition.TRANSITION_CONFIGURE,
            )

        self.get_logger().info(f"Loading map via service: {request.map_url}")
        if not self.map_load_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service /map_server/load_map unavailable")
            response.result = LoadMap.Response.RESULT_UNDEFINED_FAILURE
            return response
        fut = self.map_load_client.call_async(request)
        rclpy.spin_until_future_complete(self, fut)
        result = fut.result()
        if result is None or result.result != LoadMap.Response.RESULT_SUCCESS:
            self.get_logger().error("LoadMap call failed")
            response.result = (
                result.result if result else LoadMap.Response.RESULT_UNDEFINED_FAILURE
            )
            return response

        # Activa map_server y reactiva SLAM
        self._call_change_state(
            self.map_lifecycle_client,
            "/map_server/change_state",
            Transition.TRANSITION_ACTIVATE,
        )
        self._call_change_state(
            self.slam_lifecycle_client,
            "/slam_toolbox/change_state",
            Transition.TRANSITION_ACTIVATE,
        )

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
