from pathlib import Path
from datetime import datetime as dt
from .model import Map
from app.core.config import settings

MAP_DIR = Path(settings.maps_dir)

async def save(name: str, pgm_path: Path, yaml_path: Path):
    doc = Map(name=name, pgm=pgm_path.name,
              yaml=yaml_path.name, created=dt.utcnow())
    await doc.insert()
    return doc
