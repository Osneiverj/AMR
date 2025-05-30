from .model import Point
from app.domain.maps.model import Map
from beanie.operators import In

async def create_point(data: dict) -> Point:
    # asegura que el mapa exista
    m = await Map.get(data["map"])
    p = Point(**data, map=m)
    await p.insert()
    return p

async def list_points(map_id: str | None = None):
    if map_id:
        return await Point.find(Point.map.id == map_id).to_list()
    return await Point.find_all().to_list()

async def delete_point(point_id: str):
    p = await Point.get(point_id)
    if p: await p.delete()
    return p
