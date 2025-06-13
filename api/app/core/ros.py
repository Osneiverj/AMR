import roslibpy
import logging
from .config import settings

logger = logging.getLogger(__name__)

class RosClientManager:
    def __init__(self):
        self.client = None
        self.is_connecting = False

    def connect(self):
        if self.client and self.client.is_connected:
            logger.info("ROS client ya está conectado.")
            return

        if self.is_connecting:
            logger.warning("Conexión ROS ya en progreso.")
            return
        try:
            self.is_connecting = True
            host = settings.rosbridge_url.split('//')[1].split(':')[0]
            port = int(settings.rosbridge_url.split(':')[-1])
            self.client = roslibpy.Ros(host=host, port=port)
            self.client.run()
            logger.info(f"Conectado a ROSBridge en {settings.rosbridge_url}")
        except Exception as e:
            logger.error(f"Fallo al conectar con ROSBridge: {e}", exc_info=True)
            self.client = None
        finally:
            self.is_connecting = False

    def disconnect(self):
        if self.client and self.client.is_connected:
            self.client.terminate()
            logger.info("Conexión con ROSBridge terminada.")

    def get_client(self):
        if not self.client or not self.client.is_connected:
            logger.error("Se solicitó el cliente ROS pero no está conectado.")
            raise ConnectionError("No hay conexión con ROS.")
        return self.client

ros_manager = RosClientManager()
