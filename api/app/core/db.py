# api/app/core/db.py
from motor.motor_asyncio import AsyncIOMotorClient
from beanie import init_beanie, Document # Document ya debería estar importado de la corrección anterior

# Importa tus modelos Beanie explícitamente
from app.domain.maps.model import Map
from app.domain.missions.model import Mission
from app.domain.points.model import Point
# Añade aquí cualquier otro modelo Beanie que crees en el futuro
# from app.domain.otro_modulo.model import OtroModelo

from .config import settings

async def init_db():
    client = AsyncIOMotorClient(settings.mongo_uri)

    document_models = [
        Map,
        Mission,
        Point,
        # OtroModelo,
        # ...y así sucesivamente
    ]

    if not document_models:
        print("ADVERTENCIA: No se definieron modelos de documentos Beanie para inicializar.")
        # Considera lanzar un error si tu aplicación siempre espera modelos:
        # raise RuntimeError("No se encontraron modelos de Beanie para inicializar.")
    else:
        print(f"Inicializando Beanie con los siguientes modelos: {[model.__name__ for model in document_models]}")

    await init_beanie(
        database=client.get_default_database(), # O client[settings.mongo_db_name]
        document_models=document_models
    )