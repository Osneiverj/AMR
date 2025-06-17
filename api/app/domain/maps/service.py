# api/app/domain/maps/service.py
import logging
from pathlib import Path
from .model import Map
from app.core.config import settings
import roslibpy
import asyncio
from app.core.ros import ros_manager

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
    """Activa el modo navegación cargando el mapa a través del Orchestrator."""
    map_yaml_path = MAP_DIR / f"{map_name}.yaml"
    if not map_yaml_path.exists():
        logger.warning(f"Intento de activar un mapa inexistente: {map_name}")
        raise FileNotFoundError(f"El mapa '{map_name}' no existe en el servidor.")

    try:
        ros_client = ros_manager.get_client()
        service = roslibpy.Service(
            ros_client,
            '/ui/start_navigation_mode',
            'nav2_msgs/LoadMap'
        )
        request = roslibpy.ServiceRequest({'map_url': f"/root/maps/{map_name}.yaml"})

        loop = asyncio.get_running_loop()
        response = await loop.run_in_executor(None, lambda: service.call(request, timeout=10))

        if response is None:
            raise ConnectionError(
                "La llamada al servicio /ui/start_navigation_mode no obtuvo respuesta (timeout)."
            )

        logger.info(f"Respuesta del Orchestrator: {response}")
        return response

    except ConnectionError as e:
        logger.error(f"Error de conexión con ROS: {e}", exc_info=True)
        raise
    except Exception as e:
        logger.error(f"Error al activar el mapa '{map_name}': {e}", exc_info=True)
        raise

