# api/app/core/logging_config.py
import logging
import json
from datetime import datetime
import sys # Para sys.stdout

# Importa tus settings para configurar el nivel de log desde el entorno si es necesario
from .config import settings # Asumiendo que tienes settings.LOG_LEVEL o similar

class JsonFormatter(logging.Formatter):
    """
    Formateador para convertir los registros de log en cadenas JSON.
    """
    def format(self, record: logging.LogRecord) -> str:
        log_object = {
            "timestamp": datetime.utcfromtimestamp(record.created).isoformat() + "Z",
            "level": record.levelname,
            "message": record.getMessage(),
            "logger_name": record.name,
            # Información adicional útil para rastrear el origen del log
            "module": record.module,
            "funcName": record.funcName,
            "lineno": record.lineno,
        }
        # Añadir traceback si existe
        if record.exc_info:
            # No es ideal serializar exc_info directamente a JSON si es complejo,
            # self.formatException lo convierte en una cadena de texto (traceback).
            log_object["exception"] = self.formatException(record.exc_info)

        # Añadir campos extra si se pasan al logger
        # (ej. logger.info("mensaje", extra={"props": {"clave": "valor"}}))
        # Nos aseguramos de que los campos estándar no se sobrescriban y
        # que los "extra" no sean los atributos internos del LogRecord.
        standard_attrs = ["args", "asctime", "created", "exc_info", "exc_text", "filename",
                          "funcName", "levelname", "levelno", "lineno", "module", "msecs",
                          "message", "msg", "name", "pathname", "process", "processName",
                          "relativeCreated", "stack_info", "thread", "threadName"]

        if hasattr(record, '__dict__'):
            for key, value in record.__dict__.items():
                if key not in standard_attrs and key not in log_object:
                    log_object[key] = value

        return json.dumps(log_object)

def setup_logging_config():
    """
    Configura el logging para la aplicación.
    Limpia handlers existentes para evitar duplicación si se llama múltiples veces (poco probable con FastAPI).
    """
    # Limpiar handlers del logger raíz para evitar duplicados en recargas (más relevante para scripts)
    # En FastAPI, esto generalmente se ejecuta una vez al inicio.
    # for handler in logging.root.handlers[:]:
    #     logging.root.removeHandler(handler)
    #     handler.close()

    log_level_str = getattr(settings, 'LOG_LEVEL', 'INFO').upper()
    log_level = logging.getLevelName(log_level_str) # Convierte string a nivel de logging

    # Configurar el logger raíz
    root_logger = logging.getLogger()

    # Solo configurar si no hay handlers ya (evita reconfiguración en algunos escenarios de testing o imports)
    if not root_logger.hasHandlers():
        root_logger.setLevel(log_level)

        # Handler para la consola con formato JSON
        console_handler = logging.StreamHandler(sys.stdout) # Usar sys.stdout es buena práctica
        formatter = JsonFormatter()
        console_handler.setFormatter(formatter)
        root_logger.addHandler(console_handler)

        # Configurar niveles para loggers de librerías comunes para reducir verbosidad
        logging.getLogger("uvicorn.access").setLevel(logging.WARNING if log_level_str != "DEBUG" else logging.DEBUG)
        logging.getLogger("uvicorn.error").setLevel(log_level) # Que use el mismo nivel que la app
        logging.getLogger("uvicorn").setLevel(log_level)

        # Para Beanie/Motor si se vuelven muy ruidosos
        logging.getLogger("motor").setLevel(logging.WARNING if log_level_str != "DEBUG" else logging.INFO)
        logging.getLogger("beanie").setLevel(logging.WARNING if log_level_str != "DEBUG" else logging.INFO)

        root_logger.info(
            "Configuración de logging completada.",
            extra={"log_level": log_level_str} # 'extra' para pasar al JsonFormatter
        )
    else:
        root_logger.info("El logging ya estaba configurado.")

# --- Para usar con FastAPI ---
# Opcional: Un diccionario de configuración para pasarlo a logging.config.dictConfig
# Esto es más avanzado y permite una configuración más declarativa.
# Por ahora, la configuración programática de arriba es suficiente.
"""
LOGGING_CONFIG = {
    "version": 1,
    "disable_existing_loggers": False, # Importante para no silenciar loggers de librerías
    "formatters": {
        "json": {
            "()": JsonFormatter, # Llama a nuestra clase
        },
    },
    "handlers": {
        "console": {
            "class": "logging.StreamHandler",
            "formatter": "json",
            "level": "DEBUG", # O tomar de settings.LOG_LEVEL
            "stream": "ext://sys.stdout",
        },
    },
    "root": { # Configura el logger raíz
        "handlers": ["console"],
        "level": "INFO", # O tomar de settings.LOG_LEVEL
    },
    "loggers": { # Configuración específica para otros loggers
        "uvicorn.access": {
            "handlers": ["console"],
            "level": "WARNING",
            "propagate": False, # No pasar a handlers del logger raíz
        },
        "uvicorn.error": {
            "handlers": ["console"],
            "level": "INFO", # O settings.LOG_LEVEL
            "propagate": False,
        },
         "beanie": {
            "handlers": ["console"],
            "level": "WARNING",
            "propagate": False,
        }
    },
}
# Si usaras dictConfig:
# from logging.config import dictConfig
# dictConfig(LOGGING_CONFIG)
"""