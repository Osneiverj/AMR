# api/app/domain/missions/service.py
from beanie import PydanticObjectId
from typing import List
from .model import Mission
from app.domain.maps.model import Map # Necesario para la validación

# ------------------------------------------------------------------
# CREATE
# ------------------------------------------------------------------
async def create(data: dict) -> Mission:
    # import tardío → evita ciclo (points depende de mapas, etc.)
    # Si Point es necesario para validación, debería ser importado
    from app.domain.points.model import Point

    # 1. validar que el mapa existe
    if not await Map.get(data["map_id"]):
        raise ValueError(f"Mapa inexistente con ID: {data['map_id']}")

    # 2. validar que todos los IDs de puntos existen
    # Es más eficiente hacer una sola consulta para todos los puntos si es posible
    # o iterar como lo tienes.
    missing_points: List[str] = []
    if "sequence" in data and isinstance(data["sequence"], list):
        # Comprobar puntos solo si la secuencia existe y es una lista
        # Esto podría ser una consulta más eficiente:
        # existing_points_count = await Point.find(In(Point.id, [PydanticObjectId(pid) for pid in data["sequence"]])).count()
        # if existing_points_count != len(data["sequence"]):
        # (Esto requeriría ajustar cómo se manejan los IDs y PydanticObjectId)

        # Tu enfoque actual es correcto para la validación individual:
        for point_id_str in data["sequence"]:
            try:
                # Beanie espera PydanticObjectId para .get(), no str directamente para IDs MongoDB.
                # Sin embargo, tu modelo Point usa str para _id. Beanie podría manejarlo,
                # pero es bueno ser explícito o asegurar que el modelo Point use PydanticObjectId para su id si es el caso.
                # Asumiendo que Point.get() puede manejar str si el _id es str.
                if not await Point.get(point_id_str): # Si Point._id es str
                    missing_points.append(point_id_str)
            except Exception: # Captura errores de conversión de ID si Point.get espera PydanticObjectId
                missing_points.append(point_id_str)

    if missing_points:
        raise ValueError(f"Puntos inexistentes en la secuencia: {', '.join(missing_points)}")

    # 3. crear documento
    # Los campos como 'created' serán manejados por el modelo Beanie si tienen default_factory.
    doc = Mission(**data)
    await doc.insert()
    return doc


# ------------------------------------------------------------------
# READ (list)
# ------------------------------------------------------------------
async def list_all(map_id: str | None = None) -> List[Mission]: # Pydantic v1 compatible
    if map_id:
        return await Mission.find(Mission.map_id == map_id).to_list()
    return await Mission.find_all().to_list()


# ------------------------------------------------------------------
# DELETE
# ------------------------------------------------------------------
async def remove(mission_id: PydanticObjectId) -> bool: # Devuelve booleano
    doc = await Mission.get(mission_id)
    if not doc:
        return False
    await doc.delete()
    return True