import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_srvs.srv import Empty
from nav2_msgs.srv import LoadMap, ManageLifecycleNodes

from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State
from slam_toolbox.srv import Clear, Pause



class Orchestrator(LifecycleNode):
    def __init__(self):
        super().__init__("ui_orchestrator")

        # Grupo reentrante para evitar deadlocks en callbacks anidados
        self.cb_group = ReentrantCallbackGroup()

        # Servicios expuestos a la UI (todos usan el mismo grupo)
        self.create_service(Empty, "ui/start_slam", self.start_slam, callback_group=self.cb_group)
        self.create_service(Empty, "ui/stop_slam", self.stop_slam, callback_group=self.cb_group)
        self.create_service(LoadMap, "ui/load_map", self.load_map, callback_group=self.cb_group)
        # Apagar SLAM definitivamente una vez guardado el mapa
        self.create_service(Empty, "ui/shutdown_slam", self.shutdown_slam, callback_group=self.cb_group)
        self.create_service(Empty, "ui/start_nav2", self.start_nav2, callback_group=self.cb_group)
        self.create_service(Empty, "ui/stop_nav2", self.stop_nav2, callback_group=self.cb_group)

        # Clientes a servicios de SLAM Toolbox (no lifecycle)
        self.slam_clear_client = self.create_client(
            Clear, "/slam_toolbox/clear", callback_group=self.cb_group
        )
        self.slam_pause_client = self.create_client(
            Pause, "/slam_toolbox/pause_new_measurements", callback_group=self.cb_group
        )
        # Lifecycle para apagar slam_toolbox
        self.slam_lifecycle_client = self.create_client(
            ChangeState, "/slam_toolbox/change_state", callback_group=self.cb_group
        )

        # Cliente de servicio para Map Server + lifecycle para garantizar configuración
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

        # Arrancar la pila de navegación nada más inicializar el nodo
        self.get_logger().info("Starting Nav2 stack on startup")
        self._call_manage(
            self.nav2_lifecycle_client,
            "/lifecycle_manager_navigation/manage_nodes",
            ManageLifecycleNodes.Request.STARTUP,
        )


    # ---------------------- Helper methods ----------------------
    def _call_change_state(self, client, name: str, transition: int) -> bool:
        if not client.wait_for_service(timeout_sec=30.0):
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
        if not client.wait_for_service(timeout_sec=30.0):
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
        # Espera hasta 90 s a que el servicio aparezca (Nav2 puede tardar en arrancar)
        total_wait = 0.0
        while not client.wait_for_service(timeout_sec=5.0):
            total_wait += 5.0
            if total_wait >= 90.0:
                self.get_logger().error(f"Service {name} unavailable after 90s")
                # Debug opcional: listar servicios existentes
                import subprocess
                try:
                    services = subprocess.check_output(['ros2', 'service', 'list'], text=True)
                    self.get_logger().error(f"Servicios ROS activos:\n{services}")
                except Exception as e:
                    self.get_logger().error(f"No se pudo listar servicios: {e}")
                return None
            # Continúa esperando hasta alcanzar el límite
            continue
        req = GetState.Request()
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        if res is None:
            self.get_logger().error(f"Failed to get state from {name}")
            return None
        return res.current_state.id

    def _call_clear(self) -> bool:
        if not self.slam_clear_client.wait_for_service(timeout_sec=30.0):
            self.get_logger().error("Service /slam_toolbox/clear unavailable")
            return False
        req = Clear.Request()
        fut = self.slam_clear_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        if res and res.result == 0:          # RESULT_SUCCESS == 0
            return True
        self.get_logger().error("Clear service failed")
        return False

    def _shutdown_slam(self) -> bool:
        """Deactivate and shutdown slam_toolbox lifecycle node."""
        # If the lifecycle service is not available, assume SLAM Toolbox has already been
        # terminated (e.g., by its own LifecycleManager) and treat this as a success so
        # the orchestrator pipeline can continue without spurious errors.
        if not self.slam_lifecycle_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn("/slam_toolbox/change_state service not available – assuming SLAM Toolbox is already shutdown")
            return True
        # Deactivate if active
        self._call_change_state(
            self.slam_lifecycle_client,
            "/slam_toolbox/change_state",
            Transition.TRANSITION_DEACTIVATE,
        )
        # Shutdown
        return self._call_change_state(
            self.slam_lifecycle_client,
            "/slam_toolbox/change_state",
            Transition.TRANSITION_SHUTDOWN,
        )

    def _call_pause(self) -> bool:
        """Toggle pause via /slam_toolbox/pause_new_measurements."""
        if not self.slam_pause_client.wait_for_service(timeout_sec=3.0):
            # If the pause service is not available, we assume SLAM Toolbox is not running
            # (e.g. because the system was launched with use_slam:=False) and treat this
            # as a successful no-op so the orchestrator can proceed without blocking.
            self.get_logger().warn("/slam_toolbox/pause_new_measurements service not available – assuming SLAM disabled")
            return True
        req = Pause.Request()
        fut = self.slam_pause_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        return bool(res and res.status)

    # ---------------------- SLAM / Nav2 control ----------------------
    def start_slam(self, request, response):
        # Ensure nav2 stack is down before mapping
        # Ya no apagamos Nav2 por completo; mantener sus nodos vivos evita
        # que desaparezcan los servicios de map_server necesarios para
        # futuras llamadas.
        self.get_logger().info("Pausing navigation stack for mapping (Nav2 se mantiene activo)")

        # Sólo limpiamos SLAM; no tocamos map_server lifecycle
        self._call_clear()
        return response

    def stop_slam(self, request, response):
        self.get_logger().info("Pausing SLAM Toolbox")
        self._call_pause()
        return response

    def load_map(self, request, response):
        # Switch from mapping to navigation: stop SLAM, reset map server, load map, start Nav2
        self.get_logger().info("Pausing SLAM Toolbox before loading map")
        self._call_pause()

        # Asegura que map_server esté activo antes de llamar /load_map
        if self.map_lifecycle_client.wait_for_service(timeout_sec=5.0):
            current = self._get_state(
                self.map_state_client, "/map_server/get_state"
            )
            if current is None:
                self.get_logger().error("Cannot read map_server state")
            elif current != State.PRIMARY_STATE_ACTIVE:
                # Si no está activo, recorrer transiciones necesarias (configure+activate)
                if current == State.PRIMARY_STATE_UNCONFIGURED:
                    self._call_change_state(
                        self.map_lifecycle_client,
                        "/map_server/change_state",
                        Transition.TRANSITION_CONFIGURE,
                    )
                # Desde INACTIVE o tras configurar => activar
                self._call_change_state(
                    self.map_lifecycle_client,
                    "/map_server/change_state",
                    Transition.TRANSITION_ACTIVATE,
                )

        self.get_logger().info(f"Loading map via service: {request.map_url}")
        if not self.map_load_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("Service /map_server/load_map unavailable (timeout 10s)")
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

        # SLAM permanece en pausa; no lo reanudamos aquí

        self.get_logger().info("Resuming Nav2 stack after map load")
        self._call_manage(
            self.nav2_lifecycle_client,
            "/lifecycle_manager_navigation/manage_nodes",
            ManageLifecycleNodes.Request.RESUME,
        )
        # Apagamos slam_toolbox completamente para liberar recursos
        self._shutdown_slam()

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

    def shutdown_slam(self, request, response):
        self.get_logger().info("Shutting down SLAM Toolbox")
        ok = self._shutdown_slam()
        if not ok:
            self.get_logger().error("Failed to shutdown SLAM Toolbox")
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
