import rclpy
from rclpy.lifecycle import LifecycleNode

from std_srvs.srv import Empty, Trigger
from nav2_msgs.srv import LoadMap, ManageLifecycleNodes


class Orchestrator(LifecycleNode):
    def __init__(self):
        super().__init__("ui_orchestrator")

        # Servicios expuestos a la UI
        self.create_service(Empty, "ui/start_slam", self.start_slam)
        self.create_service(Empty, "ui/stop_slam", self.stop_slam)
        self.create_service(LoadMap, "ui/load_map", self.load_map)
        self.create_service(Empty, "ui/start_nav2", self.start_nav2)
        self.create_service(Empty, "ui/stop_nav2", self.stop_nav2)

        # Clientes para SLAM Toolbox
        self.slam_start_client = self.create_client(
            Empty, "/slam_toolbox/start"
        )
        self.slam_stop_client = self.create_client(Empty, "/slam_toolbox/stop")

        # Cliente para map_server y su servicio load_map
        self.map_load_client = self.create_client(
            LoadMap, "/map_server/load_map"
        )

        # Lifecycle managers
        self.map_lifecycle_client = self.create_client(
            ManageLifecycleNodes, "/lifecycle_manager_map_server/manage_nodes"
        )
        self.nav2_lifecycle_client = self.create_client(
            ManageLifecycleNodes, "/lifecycle_manager_navigation/manage_nodes"
        )

        self.map_active_client = self.create_client(
            Trigger, "/lifecycle_manager_map_server/is_active"
        )
        self.nav2_active_client = self.create_client(
            Trigger, "/lifecycle_manager_navigation/is_active"
        )

    # ---------------------- Helper methods ----------------------
    def _call_empty(self, client, name: str) -> bool:
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"Service {name} unavailable")
            return False
        fut = client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, fut)
        if fut.result() is None:
            self.get_logger().error(f"Call to {name} failed")
            return False
        return True

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

    def _is_active(self, client, name: str) -> bool:
        if not client.wait_for_service(timeout_sec=3.0):
            return False
        fut = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        if res is None:
            self.get_logger().warning(f"Failed to check {name} state")
            return False
        return res.success

    # ---------------------- SLAM / Nav2 control ----------------------
    def start_slam(self, request, response):
        # Ensure nav2 and map server are down before mapping
        self.get_logger().info("Stopping Nav2 stack for mapping")
        if self._is_active(
            self.nav2_active_client, "/lifecycle_manager_navigation/is_active"
        ):
            self._call_manage(
                self.nav2_lifecycle_client,
                "/lifecycle_manager_navigation/manage_nodes",
                ManageLifecycleNodes.Request.SHUTDOWN,
            )
        self.get_logger().info("Shutting down Map Server")
        if self._is_active(
            self.map_active_client, "/lifecycle_manager_map_server/is_active"
        ):
            self._call_manage(
                self.map_lifecycle_client,
                "/lifecycle_manager_map_server/manage_nodes",
                ManageLifecycleNodes.Request.SHUTDOWN,
            )
        self.get_logger().info("Starting SLAM Toolbox")
        self._call_empty(self.slam_start_client, "/slam_toolbox/start")
        return response

    def stop_slam(self, request, response):
        self.get_logger().info("Stopping SLAM Toolbox")
        self._call_empty(self.slam_stop_client, "/slam_toolbox/stop")
        return response

    def load_map(self, request, response):
        # Switch from mapping to navigation: stop SLAM, start map server, load map, start Nav2
        self.get_logger().info("Stopping SLAM Toolbox before loading map")
        self._call_empty(self.slam_stop_client, "/slam_toolbox/stop")

        if not self._is_active(
            self.map_active_client, "/lifecycle_manager_map_server/is_active"
        ):
            self.get_logger().info("Starting Map Server")
            self._call_manage(
                self.map_lifecycle_client,
                "/lifecycle_manager_map_server/manage_nodes",
                ManageLifecycleNodes.Request.STARTUP,
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
        if not self._is_active(
            self.nav2_active_client, "/lifecycle_manager_navigation/is_active"
        ):
            self._call_manage(
                self.nav2_lifecycle_client,
                "/lifecycle_manager_navigation/manage_nodes",
                ManageLifecycleNodes.Request.STARTUP,
            )
        return response

    def stop_nav2(self, request, response):
        self.get_logger().info("Shutting down Nav2 stack")
        if self._is_active(
            self.nav2_active_client, "/lifecycle_manager_navigation/is_active"
        ):
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
