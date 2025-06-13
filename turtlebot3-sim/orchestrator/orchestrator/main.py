import rclpy
from rclpy.lifecycle import LifecycleNode
from std_srvs.srv import Empty
from nav2_msgs.srv import LoadMap, ManageLifecycleNodes

class Orchestrator(LifecycleNode):
    def __init__(self):
        super().__init__('ui_orchestrator')

        # HTTP UI → ROS services
        self.create_service(Empty,   'ui/start_slam', self.start_slam)
        self.create_service(Empty,   'ui/stop_slam',  self.stop_slam)
        self.create_service(LoadMap, 'ui/load_map',   self.load_map)
        self.create_service(Empty,   'ui/start_nav2', self.start_nav2)
        self.create_service(Empty,   'ui/stop_nav2',  self.stop_nav2)

        # Clients for SLAM Toolbox (std_srvs/Empty)
        self.slam_start_client = self.create_client(Empty, '/slam_toolbox/start')
        self.slam_stop_client  = self.create_client(Empty, '/slam_toolbox/stop')

        # Client for LoadMap (nav2_msgs/LoadMap)
        self.map_load_client = self.create_client(LoadMap, '/map_server/load_map')

        # Lifecycle manager clients (nav2_msgs/ManageLifecycleNodes)
        self.map_lifecycle_client  = self.create_client(
            ManageLifecycleNodes,
            '/lifecycle_manager_map_server/manage_nodes'
        )
        self.nav2_lifecycle_client = self.create_client(
            ManageLifecycleNodes,
            '/lifecycle_manager_navigation/manage_nodes'
        )

    def call_empty(self, client, service_name: str) -> bool:
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'Service {service_name} unavailable')
            return False
        req = Empty.Request()
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        if fut.result() is None:
            self.get_logger().error(f'Empty call to {service_name} failed')
            return False
        return True

    def call_manage_lifecycle(self, client, service_name: str, command: int) -> bool:
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'Lifecycle service {service_name} unavailable')
            return False
        req = ManageLifecycleNodes.Request()
        req.command = command
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        if res and res.success:
            return True
        self.get_logger().error(f'Lifecycle {command} on {service_name} failed')
        return False

    def start_slam(self, request, response):
        # Before starting SLAM, shut down the map server if active
        self.get_logger().info('Shutting down Map Server for SLAM')
        self.call_manage_lifecycle(
            self.map_lifecycle_client,
            '/lifecycle_manager_map_server/manage_nodes',
            ManageLifecycleNodes.Request.SHUTDOWN
        )
        # Start SLAM Toolbox
        self.get_logger().info('Starting SLAM Toolbox')
        if not self.call_empty(self.slam_start_client, '/slam_toolbox/start'):
            self.get_logger().error('Failed to start SLAM Toolbox')
        return response

    def stop_slam(self, request, response):
        self.get_logger().info('Stopping SLAM Toolbox')
        self.call_empty(self.slam_stop_client, '/slam_toolbox/stop')
        return response

    def load_map(self, request, response):
        # Before loading a map, stop SLAM Toolbox
        self.get_logger().info('Stopping SLAM Toolbox before loading map')
        self.call_empty(self.slam_stop_client, '/slam_toolbox/stop')

        # Start up the Map Server lifecycle
        self.get_logger().info('Starting Map Server')
        if not self.call_manage_lifecycle(
            self.map_lifecycle_client,
            '/lifecycle_manager_map_server/manage_nodes',
            ManageLifecycleNodes.Request.STARTUP
        ):
            return response

        # Call the LoadMap service
        self.get_logger().info('Calling /map_server/load_map')
        if not self.map_load_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Map server service not available')
            return response
        fut = self.map_load_client.call_async(request)
        rclpy.spin_until_future_complete(self, fut)
        result = fut.result()
        if result is None:
            self.get_logger().error('LoadMap call failed')
            return response
        return result

    def start_nav2(self, request, response):
        # Startup the full Nav2 stack
        self.get_logger().info('Starting Nav2 stack')
        self.call_manage_lifecycle(
            self.nav2_lifecycle_client,
            '/lifecycle_manager_navigation/manage_nodes',
            ManageLifecycleNodes.Request.STARTUP
        )
        return response

    def stop_nav2(self, request, response):
        # Shutdown the Nav2 stack
        self.get_logger().info('Shutting down Nav2 stack')
        self.call_manage_lifecycle(
            self.nav2_lifecycle_client,
            '/lifecycle_manager_navigation/manage_nodes',
            ManageLifecycleNodes.Request.SHUTDOWN
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = Orchestrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()