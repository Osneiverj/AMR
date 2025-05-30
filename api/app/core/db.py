from motor.motor_asyncio import AsyncIOMotorClient
from beanie import init_beanie
from importlib import import_module
from pathlib import Path
from .config import settings

async def init_db():
    client = AsyncIOMotorClient(settings.mongo_uri)
    models = []
    for p in Path(__file__).parents[1].joinpath("domain").rglob("model.py"):
        mod = import_module(f"app.domain.{p.parent.name}.model")
        models.extend([getattr(mod, n) for n in dir(mod) if isinstance(getattr(mod, n), type)])
    await init_beanie(client.get_default_database(), document_models=models)
