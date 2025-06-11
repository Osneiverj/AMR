import rclpy
from rclpy.lifecycle import LifecycleNode
from std_srvs.srv import Empty
from nav2_msgs.srv import LoadMap

class Orchestrator(LifecycleNode):
    def __init__(self):
        super().__init__('ui_orchestrator')

        # Servicios que exponemos para la UI
        self.create_service(Empty, 'ui/start_slam', self.start_slam)
        self.create_service(Empty, 'ui/stop_slam',  self.stop_slam)
        self.create_service(LoadMap, 'ui/load_map',  self.load_map)
        self.create_service(Empty, 'ui/start_nav2', self.start_nav2)
        self.create_service(Empty, 'ui/stop_nav2',  self.stop_nav2)

    def call_empty_service(self, service_name: str) -> bool:
        """
        Cliente genérico para servicios de tipo std_srvs/Empty.
        Retorna True si la llamada fue exitosa.
        """
        client = self.create_client(Empty, service_name)
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'Service {service_name} unavailable')
            return False
        req = Empty.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result() is not None

    def start_slam(self, request, response):
        if not self.call_empty_service('/slam_toolbox/start'):
            self.get_logger().error('Failed to start SLAM Toolbox')
        return response

    def stop_slam(self, request, response):
        if not self.call_empty_service('/slam_toolbox/stop'):
            self.get_logger().error('Failed to stop SLAM Toolbox')
        return response

    def load_map(self, request, response):
        """
        Invoca el servicio /map_server/load_map de tipo nav2_msgs/LoadMap
        para cambiar el mapa en Nav2 en tiempo de ejecución.  
        """  
        client = self.create_client(LoadMap, '/map_server/load_map')
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Map server not available')
            return response
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result is None:
            self.get_logger().error('Failed to load map')
            return response
        return result

    def start_nav2(self, request, response):
        if not self.call_empty_service('/lifecycle_manager_navigation/startup'):
            self.get_logger().error('Failed to start Nav2')
        return response

    def stop_nav2(self, request, response):
        if not self.call_empty_service('/lifecycle_manager_navigation/shutdown'):
            self.get_logger().error('Failed to stop Nav2')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = Orchestrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
