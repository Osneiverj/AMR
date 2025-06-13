# api/app/domain/maps/service.py
import logging
from pathlib import Path
from datetime import datetime
from .model import Map
from app.core.config import settings
import os
import roslibpy
import threading
import asyncio

# Obtén un logger para este módulo específico
logger = logging.getLogger(__name__) 

MAP_DIR = Path(settings.maps_dir)

async def save(original_name: str, pgm_filename: str, yaml_filename: str) -> Map:
    logger.info(
        "Intentando guardar mapa",
        extra={"props": { # 'extra' es una forma estándar de pasar datos adicionales al logger
            "map_name_requested": original_name,
            "pgm_filename_to_save": pgm_filename,
            "yaml_filename_to_save": yaml_filename
            }
        }
    )
    doc = Map(
        name=original_name,
        pgm=pgm_filename,
        yaml=yaml_filename,
        # 'created' debería ser manejado por default_factory como en Paso 5
    )
    try:
        await doc.insert()
        logger.info(
            f"Mapa '{original_name}' guardado exitosamente en DB con ID: {doc.id}",
            extra={"props": {"map_id": str(doc.id), "map_name": original_name}}
        )
        return doc
    except Exception as e:
        logger.error(
            f"Error al insertar el mapa '{original_name}' en la base de datos.",
            exc_info=True, # Captura el traceback completo
            extra={"props": {"map_name": original_name, "error_type": type(e).__name__}}
        )
        # Re-lanza la excepción o maneja de otra forma si es necesario
        raise


def list_available_maps() -> list[str]:
    """Devuelve una lista de mapas válidos en el directorio de mapas."""
    if not MAP_DIR.is_dir():
        return []

    map_files = [f.stem for f in MAP_DIR.glob('*.yaml')]
    valid_maps = [m for m in map_files if (MAP_DIR / f"{m}.pgm").exists()]
    logger.info(f"Mapas disponibles encontrados: {valid_maps}")
    return valid_maps


async def activate_map(map_name: str) -> dict:
    """Carga un mapa en ROS a través del servicio expuesto por el Orchestrator ('/ui/load_map')."""
    map_yaml_path = MAP_DIR / f"{map_name}.yaml"
    if not map_yaml_path.exists():
        logger.warning(f"Intento de activar un mapa inexistente: {map_name}")
        raise FileNotFoundError(f"El mapa '{map_name}' no existe en el servidor.")

    # Extraer host y puerto desde la URL de rosbridge
    host = settings.rosbridge_url.split('//')[1].split(':')[0]
    port = int(settings.rosbridge_url.split(':')[-1])
    ros_client = roslibpy.Ros(host=host, port=port)

    try:
        # Lanzar ros_client.run en segundo plano
        thread = threading.Thread(target=ros_client.run, daemon=True)
        thread.start()

        # Esperar hasta que rosbridge acepte la conexión (timeout 5 s)
        timeout = 5.0
        waited = 0.0
        interval = 0.1
        while not ros_client.is_connected and waited < timeout:
            await asyncio.sleep(interval)
            waited += interval

        if not ros_client.is_connected:
            raise ConnectionError("No se pudo conectar con ROS.")

        logger.info(f"Conectado a ROSbridge en {settings.rosbridge_url}")

        # Llamar al servicio UI de Orchestrator
        service = roslibpy.Service(ros_client, '/ui/load_map', 'nav2_msgs/LoadMap')
        request = roslibpy.ServiceRequest({'map_url': f"/root/maps/{map_name}.yaml"})

        loop = asyncio.get_running_loop()
        response = await loop.run_in_executor(None, lambda: service.call(request))
        logger.info(f"Respuesta del Orchestrator: {response}")
        return response

    except asyncio.TimeoutError:
        logger.error(f"Timeout al conectar con ROSbridge en {settings.rosbridge_url}")
        raise ConnectionError("No se pudo conectar con ROS.")
    except Exception as e:
        logger.error(f"Error al activar el mapa '{map_name}': {e}", exc_info=True)
        raise
    finally:
        if ros_client.is_connected:
            ros_client.terminate()
            logger.info("Conexión con ROSbridge terminada.")
