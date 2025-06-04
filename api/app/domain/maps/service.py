# api/app/domain/maps/service.py
import logging
from pathlib import Path
from datetime import datetime
from .model import Map
from app.core.config import settings

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
