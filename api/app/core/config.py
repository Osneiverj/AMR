# api/app/core/config.py
from pydantic import BaseSettings # En Pydantic V1, BaseSettings está en pydantic. Para V2 es pydantic_settings.
from typing import List, Union # Union para Pydantic V1

class Settings(BaseSettings):
    mongo_uri: str = "mongodb://mongodb:27017/agv"
    maps_dir: str  = "/maps"
    # Para Pydantic V1, Union[str, List[str]] es más típico que str | list[str]
    cors_origins: Union[str, List[str]] = "*" 

    LOG_LEVEL: str = "INFO" # Nivel de log por defecto

    default_admin_username: str = "admin"
    default_admin_password: str = "admin"

    class Config:
        env_prefix = "AGV_"
        # Para Pydantic V1, si usas .env files, necesitarías:
        # env_file = ".env" 
        # env_file_encoding = "utf-8"

settings = Settings()
