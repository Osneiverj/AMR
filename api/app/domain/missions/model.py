# api/app/domain/missions/model.py
from beanie import Document
from datetime import datetime
from typing import List
from pydantic import Field # <-- IMPORTAR Field

class Mission(Document):
    name: str
    map_id: str                      # referencia por id
    sequence: List[str]              # lista de point._id
    loop: bool = False
    created: datetime = Field(default_factory=datetime.utcnow) # <-- CAMBIADO

    class Settings:
        name = "missions"
        