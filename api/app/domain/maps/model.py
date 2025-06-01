# api/app/domain/maps/model.py
from beanie import Document
from datetime import datetime
from pydantic import Field # <-- IMPORTAR Field

class Map(Document):
    name: str         # único, nombre lógico del mapa
    pgm: str          # nombre del archivo .pgm en el filesystem
    yaml: str         # nombre del archivo .yaml en el filesystem
    created: datetime = Field(default_factory=datetime.utcnow) # <-- CAMBIADO

    class Settings:
        name = "maps"