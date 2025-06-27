import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_srvs.srv import Empty
from nav2_msgs.srv import LoadMap, ManageLifecycleNodes

from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State
from slam_toolbox_msgs.srv import Reset, Pause


class Orchestrator(LifecycleNode):
    def __init__(self):
        super().__init__("ui_orchestrator")

        # Grupo reentrante para evitar deadlocks en callbacks anidados
        self.cb_group = ReentrantCallbackGroup()

        # Servicios expuestos a la UI (todos usan el mismo grupo)
        self.create_service(Empty, "ui/start_slam", self.start_slam, callback_group=self.cb_group)
        self.create_service(Empty, "ui/stop_slam", self.stop_slam, callback_group=self.cb_group)
        self.create_service(LoadMap, "ui/load_map", self.load_map, callback_group=self.cb_group)
        self.create_service(Empty, "ui/start_nav2", self.start_nav2, callback_group=self.cb_group)
        self.create_service(Empty, "ui/stop_nav2", self.stop_nav2, callback_group=self.cb_group)

        # Clientes a servicios de SLAM Toolbox (no lifecycle)
        self.slam_reset_client = self.create_client(
            Reset, "/slam_toolbox/reset", callback_group=self.cb_group
        )
        self.slam_pause_client = self.create_client(
            Pause, "/slam_toolbox/pause_new_measurements", callback_group=self.cb_group
        )

        # Lifecycle clients para Map Server

        self.map_load_client = self.create_client(
            LoadMap, "/map_server/load_map", callback_group=self.cb_group
        )
        self.map_lifecycle_client = self.create_client(
            ChangeState, "/map_server/change_state", callback_group=self.cb_group
        )
        self.map_state_client = self.create_client(
            GetState, "/map_server/get_state", callback_group=self.cb_group
        )
        self.nav2_lifecycle_client = self.create_client(
            ManageLifecycleNodes, "/lifecycle_manager_navigation/manage_nodes", callback_group=self.cb_group
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
            # Debug: listar servicios disponibles
            import subprocess
            try:
                services = subprocess.check_output(['ros2', 'service', 'list'], text=True)
                self.get_logger().error(f"Servicios ROS activos:\n{services}")
            except Exception as e:
                self.get_logger().error(f"No se pudo listar servicios: {e}")
            return None
        req = GetState.Request()
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        if res is None:
            self.get_logger().error(f"Failed to get state from {name}")
            return None
        return res.current_state.id

    def _call_reset(self, pause: bool) -> bool:
        """Call /slam_toolbox/reset service."""
        if not self.slam_reset_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service /slam_toolbox/reset unavailable")
            return False
        req = Reset.Request()
        req.pause_new_measurements = pause
        fut = self.slam_reset_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        if res and res.result == 0:
            return True
        self.get_logger().error("Reset service failed")
        return False

    def _call_pause(self) -> bool:
        """Toggle pause via /slam_toolbox/pause_new_measurements."""
        if not self.slam_pause_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service /slam_toolbox/pause_new_measurements unavailable")
            return False
        req = Pause.Request()
        fut = self.slam_pause_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        return bool(res and res.status)

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

        # Reinicia SLAM Toolbox para comenzar un nuevo mapeo
        self._call_reset(False)
        return response

    def stop_slam(self, request, response):
        self.get_logger().info("Pausing SLAM Toolbox")
        self._call_pause()
        return response

    def load_map(self, request, response):
        # Switch from mapping to navigation: stop SLAM, reset map server, load map, start Nav2
        self.get_logger().info("Pausing SLAM Toolbox before loading map")
        self._call_pause()

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

        # Activa map_server y reanuda SLAM
        self._call_change_state(
            self.map_lifecycle_client,
            "/map_server/change_state",
            Transition.TRANSITION_ACTIVATE,
        )
        self._call_pause()

        self.get_logger().info("Starting Nav2 stack after map load")
        self._call_manage(
            self.nav2_lifecycle_client,
            "/lifecycle_manager_navigation/manage_nodes",
            ManageLifecycleNodes.Request.STARTUP,
        )
        response.map = result.map
        response.result = result.result
        return response

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
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
