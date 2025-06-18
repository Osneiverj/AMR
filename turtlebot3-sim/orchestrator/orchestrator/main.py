import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from nav2_msgs.srv import LoadMap
import sys
import os

class Orchestrator(Node):
    def __init__(self):
        super().__init__('ui_orchestrator')
        self.get_logger().info('Iniciando Orquestador de UI...')

        # --- Clientes para Nodos de Ciclo de Vida (Lifecycle Nodes) ---
        
        # Cliente para cambiar el estado del map_server
        self.map_server_change_state_client = self.create_client(ChangeState, '/map_server/change_state')
        
        # Puedes añadir más clientes para otros nodos de ciclo de vida aquí
        # self.amcl_change_state_client = self.create_client(ChangeState, '/amcl/change_state')
        # self.controller_server_change_state_client = self.create_client(ChangeState, '/controller_server/change_state')
        # ... y así sucesivamente para todos los nodos que necesites gestionar

        # --- Servidores para Servicios ROS ---

        # Servidor para el servicio de carga de mapas (CÓDIGO CORREGIDO)
        self.load_map_service = self.create_service(LoadMap, '/ui/load_map', self.load_map)

        self.get_logger().info('Orquestador de UI listo.')

    def change_map_server_state(self, transition_id):
        """
        Solicita un cambio de estado para el nodo map_server.
        """
        request = ChangeState.Request()
        request.transition.id = transition_id
        
        # Llamada asíncrona al servicio
        future = self.map_server_change_state_client.call_async(request)
        
        # Esperamos el resultado de forma síncrona en este hilo.
        # En una aplicación más compleja, podrías manejar esto de forma asíncrona.
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info(f'Transición {transition_id} del map_server completada con éxito.')
                return True
            else:
                self.get_logger().error(f'Falló la transición {transition_id} del map_server.')
                return False
        else:
            self.get_logger().error(f'No se pudo llamar al servicio de cambio de estado del map_server para la transición {transition_id}.')
            return False

    def load_map(self, request, response):
        """
        Gestiona el ciclo de vida del map_server para cargar un nuevo mapa.
        """
        self.get_logger().info(f'Solicitud para cargar el mapa: {request.map_url}')

        # 1. Esperar a que el servicio change_state del map_server esté disponible
        if not self.map_server_change_state_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Servicio /map_server/change_state no disponible.')
            response.result = LoadMap.Response.RESULT_UNDEFINED_FAILURE
            return response

        # 2. Desactivar y limpiar el map_server si ya está activo
        # Esto asegura que podamos cargar un nuevo mapa sin conflictos.
        self.get_logger().info('Desactivando y limpiando Map Server (si es necesario)...')
        self.change_map_server_state(Transition.TRANSITION_DEACTIVATE)
        self.change_map_server_state(Transition.TRANSITION_CLEANUP)
        
        # 3. Actualizar el parámetro `yaml_filename` del map_server
        self.get_logger().info(f'Actualizando el parámetro yaml_filename a: {request.map_url}')
        # NOTA: Esto requiere que el nodo `map_server` tenga el parámetro declarado y que
        # el orquestador tenga permisos para cambiarlo.
        # Aquí usamos un método simple, pero en un sistema real podrías usar el servicio de parámetros.
        os.system(f'ros2 param set /map_server yaml_filename {request.map_url}')

        # 4. Configurar el map_server
        self.get_logger().info('Configurando Map Server...')
        if not self.change_map_server_state(Transition.TRANSITION_CONFIGURE):
            response.result = LoadMap.Response.RESULT_UNDEFINED_FAILURE
            return response

        # 5. Activar el map_server para que cargue el mapa y empiece a publicarlo
        self.get_logger().info('Activando Map Server...')
        if not self.change_map_server_state(Transition.TRANSITION_ACTIVATE):
            response.result = LoadMap.Response.RESULT_UNDEFINED_FAILURE
            return response

        self.get_logger().info(f"Llamando a /map_server/load_map con URL: {request.map_url}")
        if not self.map_load_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Servicio /map_server/load_map no disponible tras la activación.')
            response.result = LoadMap.Response.RESULT_UNDEFINED_FAILURE
            return response

        fut = self.map_load_client.call_async(request)
        rclpy.spin_until_future_complete(self, fut)
        result = fut.result()

        if result is None:
            self.get_logger().error('La llamada a LoadMap falló (timeout o error interno).')
            response.result = LoadMap.Response.RESULT_UNDEFINED_FAILURE
            return response

        self.get_logger().info(f'Respuesta de LoadMap: {result}')
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

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = Orchestrator()
        # Usamos un ejecutor Multi-Hilos para que las llamadas a servicios
        # no bloqueen el procesamiento de otros callbacks.
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            node.destroy_node()
            
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()