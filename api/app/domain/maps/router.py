# api/app/domain/maps/router.py
from fastapi import APIRouter, UploadFile, HTTPException, File, Form, Depends # File y Form para multipart
from .service import save, MAP_DIR, list_available_maps, activate_map
from .model import Map
import shutil
from app.domain.users.service import get_current_active_user, require_admin
from typing import Annotated, List

router = APIRouter(prefix="/maps", tags=["maps"])

@router.get("/", response_model=list[Map])
async def list_maps(current_user = Depends(get_current_active_user)):

    return await Map.find_all().to_list()


@router.get("/available", response_model=List[str])
async def get_available_maps(current_user = Depends(get_current_active_user)):
    """Devuelve los nombres de mapas presentes en el filesystem."""
    return list_available_maps()


@router.post("/{map_name}/activate", status_code=200)
async def activate_map_endpoint(map_name: str, current_user = Depends(require_admin)):
    """Carga el mapa solicitado y arranca el modo navegación en ROS."""
    try:
        result = await activate_map(map_name)
        if result.get('result') == 0:
            return {"message": f"Mapa '{map_name}' activado correctamente."}
        raise HTTPException(status_code=400, detail=f"Fallo al activar el mapa en ROS: {result.get('text', 'Error desconocido')}")
    except FileNotFoundError as e:
        raise HTTPException(status_code=404, detail=str(e))
    except ConnectionError as e:
        raise HTTPException(status_code=503, detail=f"Error de comunicación con ROS: {e}")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error interno del servidor: {e}")

# Para FastAPI 0.109.2, es mejor ser explícito con Form y File
@router.post("/", response_model=Map, status_code=201)
async def upload_map(
    name: Annotated[str, Form()],
    pgm: Annotated[UploadFile, File()],
    yaml: Annotated[UploadFile, File()],
    current_user = Depends(require_admin)
):
    if await Map.find_one(Map.name == name):
        raise HTTPException(status_code=409, detail="Nombre de mapa existente")

    MAP_DIR.mkdir(exist_ok=True, parents=True) # parents=True por si MAP_DIR es profundo

    # Considerar sanitizar 'name' para nombres de archivo seguros
    safe_filename_base = "".join(c if c.isalnum() or c in ('_', '-') else '_' for c in name)
    if not safe_filename_base: # Si el nombre original solo tenía caracteres no válidos
        raise HTTPException(status_code=400, detail="Nombre de mapa inválido para generar nombre de archivo.")

    pgm_path = MAP_DIR / f"{safe_filename_base}.pgm"
    yaml_path = MAP_DIR / f"{safe_filename_base}.yaml"

    try:
        with pgm_path.open("wb") as f_pgm:
            shutil.copyfileobj(pgm.file, f_pgm)
        with yaml_path.open("wb") as f_yaml:
            shutil.copyfileobj(yaml.file, f_yaml)
    except IOError as e:
        # Limpiar archivos si uno falla, para evitar estado inconsistente
        if pgm_path.exists():
            pgm_path.unlink(missing_ok=True)
        if yaml_path.exists():
            yaml_path.unlink(missing_ok=True)
        raise HTTPException(status_code=500, detail=f"Error al guardar archivos de mapa: {e}")
    finally:
        pgm.file.close()
        yaml.file.close()

    # El servicio save ahora recibe los nombres de archivo correctos (safe_filename_base.pgm/yaml)
    # y el nombre original para el documento.
    return await save(original_name=name, 
                      pgm_filename=f"{safe_filename_base}.pgm", 
                      yaml_filename=f"{safe_filename_base}.yaml")
