import rclpy
from rclpy.lifecycle import LifecycleNode
from lifecycle_msgs.msg import Transition
from nav2_msgs.srv import LoadMap
from std_srvs.srv import Empty
from slam_toolbox_msgs.srv import LifecycleControl

class Orchestrator(LifecycleNode):
    def __init__(self):
        super().__init__('ui_orchestrator')
        self.create_service(Empty, 'ui/start_slam', self.start_slam)
        self.create_service(Empty, 'ui/stop_slam', self.stop_slam)
        self.create_service(LoadMap, 'ui/load_map', self.load_map)
        self.create_service(Empty, 'ui/start_nav2', self.start_nav2)
        self.create_service(Empty, 'ui/stop_nav2', self.stop_nav2)

    def call_empty_service(self, service_name):
        client = self.create_client(Empty, service_name)
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'Service {service_name} unavailable')
            return
        request = Empty.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

    def start_slam(self, request, response):
        self.call_empty_service('/slam_toolbox/start')
        return response

    def stop_slam(self, request, response):
        self.call_empty_service('/slam_toolbox/stop')
        return response

    def load_map(self, request, response):
        client = self.create_client(LoadMap, '/map_server/load_map')
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Map server not available')
            return response
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def start_nav2(self, request, response):
        self.call_empty_service('/lifecycle_manager_navigation/startup')
        return response

    def stop_nav2(self, request, response):
        self.call_empty_service('/lifecycle_manager_navigation/shutdown')
        return response

def main():
    rclpy.init()
    node = Orchestrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
