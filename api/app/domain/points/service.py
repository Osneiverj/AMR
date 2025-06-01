# api/app/domain/points/service.py
from .model import Point
from app.domain.maps.model import Map # Asumiendo que Map está en este path

async def create_point(data: dict) -> Point:
    # Asegura que el mapa exista
    map_document = await Map.get(data["map_id"])
    if not map_document:
        raise ValueError(f"Mapa con ID {data['map_id']} no encontrado.")

    # Crea la instancia del modelo Beanie Point
    # Los campos como 'created' serán manejados por el modelo Beanie
    # si tienen default_factory.
    point_to_create = Point(**data)
    await point_to_create.insert()
    return point_to_create

async def list_points(map_id: str | None = None): # Pydantic v1 compatible
    if map_id:
        return await Point.find(Point.map_id == map_id).to_list()
    return await Point.find_all().to_list()

async def delete_point(point_id: str): # Cambiado para aceptar str directamente
    point = await Point.get(point_id)
    if point:
        await point.delete()
        return True # O el objeto eliminado 'point' si prefieres
    return False # O None