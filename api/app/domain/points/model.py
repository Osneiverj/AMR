# api/app/domain/points/model.py
from beanie import Document
from datetime import datetime
from typing import Optional # Para Pydantic v1, Optional[Pose] en lugar de Pose | None
from pydantic import Field # <-- IMPORTAR Field
from app.core.schemas import Pose

class Point(Document):
    name: str
    type: str                        # way | dock | station
    map_id: str                      # _id (str) del Map
    target: Pose
    face_target: Optional[Pose] = None # Pydantic v1
    created: datetime = Field(default_factory=datetime.utcnow) # <-- CAMBIADO

    class Settings:
        name = "points"
        