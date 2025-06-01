# api/app/domain/points/schemas.py
from pydantic import BaseModel
from typing import Optional # Usa Optional en lugar de | None para compatibilidad con Pydantic v1
from app.core.schemas import Pose # Importas tu esquema Pose existente

class PointCreate(BaseModel):
    name: str
    type: str  # way | dock | station
    map_id: str  # _id (str) del Map
    target: Pose
    face_target: Optional[Pose] = None # Pydantic v1 usa Optional[Pose]

class PointUpdate(BaseModel): # Ejemplo si necesitaras actualizar puntos
    name: Optional[str] = None
    type: Optional[str] = None
    map_id: Optional[str] = None
    target: Optional[Pose] = None
    face_target: Optional[Pose] = None
    # No incluyas 'created' aquí, ya que no debería ser actualizable por el cliente