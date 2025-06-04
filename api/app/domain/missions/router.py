# api/app/domain/missions/router.py
from fastapi import APIRouter, HTTPException, Depends
from beanie import PydanticObjectId
from .service import create, list_all, remove
from .model import Mission  # Sigue siendo necesario para el response_model
from .schemas import MissionCreate
from app.domain.users.service import get_current_active_user, require_admin  # <-- IMPORTA EL NUEVO ESQUEMA

router = APIRouter(prefix="/missions", tags=["missions"])

@router.get("/", response_model=list[Mission])
async def get_missions(map_id: str | None = None, current_user = Depends(get_current_active_user)): # Pydantic v1 compatible
    return await list_all(map_id)

@router.post("/", response_model=Mission, status_code=201)
async def new_mission(
    mission_data: MissionCreate,
    current_user = Depends(require_admin)
):
    try:
        # El servicio ahora recibirá un diccionario validado de mission_data
        return await create(mission_data.dict(exclude_none=True))
    except ValueError as e:
        # Errores de validación del servicio (mapa/puntos inexistentes)
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        # Otros errores inesperados
        raise HTTPException(status_code=500, detail="Error interno al crear la misión.")


@router.delete("/{mid}", status_code=204)
async def delete_mission(mid: PydanticObjectId, current_user = Depends(require_admin)):
    success = await remove(mid) # Asumiendo que remove ahora devuelve un booleano
    if not success:
        raise HTTPException(status_code=404, detail="Mission not found")
    # No se devuelve contenido en un 204
