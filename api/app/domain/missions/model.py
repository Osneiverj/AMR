# api/app/domain/missions/model.py
from beanie import Document
from datetime import datetime
from typing import List
from pydantic import Field # <-- IMPORTAR Field

class Mission(Document):
    name: str
    map_id: str                      # referencia por id
    definition: dict                 # Estructura del árbol de comportamiento
    loop: bool = False
    created: datetime = Field(default_factory=datetime.utcnow)

    class Settings:
        name = "missions"
        