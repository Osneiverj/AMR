# api/app/domain/missions/schemas.py
from pydantic import BaseModel
from typing import List, Optional # Usaremos Optional para Pydantic v1

class MissionCreate(BaseModel):
    name: str
    map_id: str  # referencia por id
    definition: dict  # Estructura del árbol de comportamiento
    loop: bool = False

class MissionUpdate(BaseModel): # Ejemplo si necesitaras actualizar misiones
    name: Optional[str] = None
    map_id: Optional[str] = None
    definition: Optional[dict] = None
    loop: Optional[bool] = None
    # No incluyas 'created' aquí