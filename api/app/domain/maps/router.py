# api/app/domain/maps/router.py
from fastapi import APIRouter, UploadFile, HTTPException, File, Form # File y Form para multipart
from .service import save, MAP_DIR
from .model import Map
import shutil
from typing import Annotated # Para FastAPI 0.109+ con Pydantic v1

router = APIRouter(prefix="/maps", tags=["maps"])

@router.get("/", response_model=list[Map])
async def list_maps():
    return await Map.find_all().to_list()

# Para FastAPI 0.109.2, es mejor ser explícito con Form y File
@router.post("/", response_model=Map, status_code=201)
async def upload_map(
    name: Annotated[str, Form()],
    pgm: Annotated[UploadFile, File()],
    yaml: Annotated[UploadFile, File()]
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