from fastapi import APIRouter, HTTPException
from beanie import PydanticObjectId
from .service import create_point, list_points, delete_point
from .model import Point

router = APIRouter(prefix="/points", tags=["points"])

@router.get("/", response_model=list[Point])
async def get_points(map_id: str | None = None):
    return await list_points(map_id)

@router.post("/", response_model=Point, status_code=201)
async def new_point(point: Point):
    return await create_point(point.dict(exclude={"id"}))

@router.delete("/{pid}", status_code=204)
async def remove_point(pid: PydanticObjectId):
    if not await delete_point(pid):
        raise HTTPException(404)
