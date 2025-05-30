from pydantic import BaseSettings

class Settings(BaseSettings):
    mongo_uri: str = "mongodb://mongodb:27017/agv"
    maps_dir: str  = "/maps"
    cors_origins: str | list[str] = "*"

    class Config:
        env_prefix = "AGV_"

settings = Settings()
