import rclpy
from rclpy.lifecycle import LifecycleNode

from std_srvs.srv import Empty
from nav2_msgs.srv import LoadMap, ManageLifecycleNodes
import os


class Orchestrator(LifecycleNode):
    def __init__(self):
        super().__init__('ui_orchestrator')

        # Servicios expuestos a la UI
        self.create_service(Empty,   'ui/start_slam', self.start_slam)
        self.create_service(Empty,   'ui/stop_slam',  self.stop_slam)
        self.create_service(LoadMap, 'ui/load_map',   self.load_map)
        self.create_service(Empty,   'ui/start_nav2', self.start_nav2)
        self.create_service(Empty,   'ui/stop_nav2',  self.stop_nav2)

        # Clientes para SLAM Toolbox
        self.slam_start_client = self.create_client(Empty, '/slam_toolbox/start')
        self.slam_stop_client  = self.create_client(Empty, '/slam_toolbox/stop')

        # Cliente para map_server y su servicio load_map
        self.map_load_client = self.create_client(LoadMap, '/map_server/load_map')

        # Lifecycle managers
        self.map_lifecycle_client = self.create_client(
            ManageLifecycleNodes,
            '/lifecycle_manager_map_server/manage_nodes')
        self.nav2_lifecycle_client = self.create_client(
            ManageLifecycleNodes,
            '/lifecycle_manager_navigation/manage_nodes')

    # ---------------------- Helper methods ----------------------
    def _call_empty(self, client, name: str) -> bool:
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(f'Service {name} unavailable')
            return False
        fut = client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, fut)
        if fut.result() is None:
            self.get_logger().warn(f'Call to {name} failed')
            return False
        return True

    def _call_manage(self, client, name: str, command: int) -> bool:
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(f'Lifecycle service {name} unavailable')
            return False
        req = ManageLifecycleNodes.Request(command=command)
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        if res and res.success:
            return True
        self.get_logger().warn(f'Lifecycle command {command} on {name} failed')
        return False

    # ---------------------- SLAM / Nav2 control ----------------------
    def start_slam(self, request, response):
        self.get_logger().info('Switching to mapping mode')
        self._call_manage(
            self.nav2_lifecycle_client,
            '/lifecycle_manager_navigation/manage_nodes',
            ManageLifecycleNodes.Request.SHUTDOWN)
        self._call_manage(
            self.map_lifecycle_client,
            '/lifecycle_manager_map_server/manage_nodes',
            ManageLifecycleNodes.Request.SHUTDOWN)
        self._call_empty(self.slam_start_client, '/slam_toolbox/start')
        return response

    def stop_slam(self, request, response):
        self.get_logger().info('Stopping SLAM Toolbox')
        self._call_empty(self.slam_stop_client, '/slam_toolbox/stop')
        return response

    def load_map(self, request, response):
        self.get_logger().info('Loading map and enabling navigation')
        self._call_empty(self.slam_stop_client, '/slam_toolbox/stop')
        self._call_manage(
            self.map_lifecycle_client,
            '/lifecycle_manager_map_server/manage_nodes',
            ManageLifecycleNodes.Request.SHUTDOWN)

        os.system(f'ros2 param set /map_server yaml_filename {request.map_url}')
        self._call_manage(
            self.map_lifecycle_client,
            '/lifecycle_manager_map_server/manage_nodes',
            ManageLifecycleNodes.Request.STARTUP)

        if not self.map_load_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('Service /map_server/load_map unavailable')
            response.result = LoadMap.Response.RESULT_FAILURE
            return response

        fut = self.map_load_client.call_async(request)
        rclpy.spin_until_future_complete(self, fut)
        result = fut.result()
        if result is None or result.result != LoadMap.Response.RESULT_SUCCESS:
            self.get_logger().warn('LoadMap call failed')
            response.result = LoadMap.Response.RESULT_FAILURE
            return response

        self._call_manage(
            self.nav2_lifecycle_client,
            '/lifecycle_manager_navigation/manage_nodes',
            ManageLifecycleNodes.Request.STARTUP)
        return result

    def start_nav2(self, request, response):
        self.get_logger().info('Starting Nav2 stack')
        self._call_manage(
            self.nav2_lifecycle_client,
            '/lifecycle_manager_navigation/manage_nodes',
            ManageLifecycleNodes.Request.STARTUP)
        return response

    def stop_nav2(self, request, response):
        self.get_logger().info('Shutting down Nav2 stack')
        self._call_manage(
            self.nav2_lifecycle_client,
            '/lifecycle_manager_navigation/manage_nodes',
            ManageLifecycleNodes.Request.SHUTDOWN)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = Orchestrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
