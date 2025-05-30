# api/app/domain/missions/router.py
from fastapi import APIRouter, HTTPException
from beanie import PydanticObjectId
from .service import create, list_all, remove   # ← ya existen
from .model import Mission

router = APIRouter(prefix="/missions", tags=["missions"])

@router.get("/", response_model=list[Mission])
async def get_missions(map_id: str | None = None):
    return await list_all(map_id)

@router.post("/", response_model=Mission, status_code=201)
async def new_mission(mission: Mission):
    try:
        return await create(mission.dict(exclude={"id"}))
    except ValueError as e:
        raise HTTPException(400, str(e))

@router.delete("/{mid}", status_code=204)
async def delete_mission(mid: PydanticObjectId):
    if not await remove(mid):
        raise HTTPException(404)
