from beanie import PydanticObjectId
from typing import List
from .model import Mission
from app.domain.maps.model import Map

# ------------------------------------------------------------------
# CREATE
# ------------------------------------------------------------------
async def create(data: dict) -> Mission:
    # import tardío → evita ciclo (points depende de mapas, etc.)
    from app.domain.points.model import Point

    # 1. validar que el mapa existe
    if not await Map.get(data["map_id"]):
        raise ValueError("Mapa inexistente")

    # 2. validar que todos los IDs de puntos existen
    missing: List[str] = [
        pid for pid in data["sequence"] if not await Point.get(pid)
    ]
    if missing:
        raise ValueError(f"Puntos inexistentes: {', '.join(missing)}")

    # 3. crear documento
    doc = Mission(**data)
    await doc.insert()
    return doc


# ------------------------------------------------------------------
# READ (list)
# ------------------------------------------------------------------
async def list_all(map_id: str | None = None) -> List[Mission]:
    if map_id:
        return await Mission.find(Mission.map_id == map_id).to_list()
    return await Mission.find_all().to_list()


# ------------------------------------------------------------------
# DELETE
# ------------------------------------------------------------------
async def remove(mission_id: PydanticObjectId) -> bool:
    doc = await Mission.get(mission_id)
    if not doc:
        return False
    await doc.delete()
    return True
