from fastapi import APIRouter, UploadFile, HTTPException
from .service import save, MAP_DIR
from .model import Map
import shutil

router = APIRouter(prefix="/maps", tags=["maps"])

@router.get("/", response_model=list[Map])
async def list_maps():
    return await Map.find_all().to_list()

@router.post("/", status_code=201)
async def upload(name: str, pgm: UploadFile, yaml: UploadFile):
    if await Map.find_one(Map.name == name):
        raise HTTPException(409, "Nombre existente")
    MAP_DIR.mkdir(exist_ok=True)
    pgm_path = MAP_DIR / f"{name}.pgm"
    yaml_path = MAP_DIR / f"{name}.yaml"
    with pgm_path.open("wb") as f: shutil.copyfileobj(pgm.file, f)
    with yaml_path.open("wb") as f: shutil.copyfileobj(yaml.file, f)
    return await save(name, pgm_path, yaml_path)
