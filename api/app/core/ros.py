import roslibpy
import os

ROSBRIDGE_URL = os.getenv("AGV_ROSBRIDGE_URL", "ws://localhost:9090")

client = roslibpy.Ros(host=ROSBRIDGE_URL.split('//')[1].split(':')[0], port=int(ROSBRIDGE_URL.split(':')[2]))

try:
    client.run()
    client.wait_for_connection()
    print("Conectado a rosbridge.")
except Exception as e:
    print(f"No se pudo conectar a rosbridge: {e}")

def get_ros_client():
    return client

ros_manager = get_ros_client()