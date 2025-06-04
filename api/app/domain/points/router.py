# api/app/domain/points/router.py
from fastapi import APIRouter, HTTPException, Depends
from beanie import PydanticObjectId
from .service import create_point, list_points, delete_point
from .model import Point  # Sigue siendo necesario para el response_model
from .schemas import PointCreate
from app.domain.users.service import get_current_active_user, require_admin  # <-- IMPORTA EL NUEVO ESQUEMA

router = APIRouter(prefix="/points", tags=["points"])

@router.get("/", response_model=list[Point])
async def get_points(map_id: str | None = None, current_user = Depends(get_current_active_user)): # Pydantic v1 compatible con str | None
    return await list_points(map_id)

@router.post("/", response_model=Point, status_code=201)
async def new_point(
    point_data: PointCreate,
    current_user = Depends(require_admin)
):  # <-- USA EL ESQUEMA PointCreate
    # El servicio ahora recibirá un diccionario validado de point_data
    return await create_point(point_data.dict(exclude_none=True)) # exclude_none es útil

@router.delete("/{pid}", status_code=204)
async def remove_point(pid: PydanticObjectId, current_user = Depends(require_admin)):
    point_deleted = await delete_point(str(pid)) # Asegúrate que delete_point espera un str si pid es PydanticObjectId
    if not point_deleted:
        raise HTTPException(status_code=404, detail="Point not found")
