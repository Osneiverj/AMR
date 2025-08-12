import asyncio
from motor.motor_asyncio import AsyncIOMotorClient
from beanie import init_beanie, Document
from pydantic import Field
from typing import List, Optional
from uuid import UUID, uuid4

# --- Modelos (copiados para que el script sea autocontenido) ---
class Map(Document):
    id: UUID = Field(default_factory=uuid4, alias="_id")
    name: str
    pgm_filename: str
    yaml_filename: str

    class Settings:
        name = "maps"

class Point(Document):
    id: UUID = Field(default_factory=uuid4, alias="_id")
    name: str
    map_id: UUID
    x: float
    y: float
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    class Settings:
        name = "points"

# --- Lógica del Script ---
async def seed():
    # Conexión a la base de datos
    client = AsyncIOMotorClient("mongodb://localhost:27017/agv") # Usamos localhost porque corremos el script desde fuera de Docker
    await init_beanie(database=client.get_default_database(), document_models=[Map, Point])

    print("Limpiando colecciones existentes...")
    await Map.delete_all()
    await Point.delete_all()

    print("Creando datos de prueba...")

    # Crear Mapa de Prueba
    test_map = Map(
        name="Laberinto de Prueba",
        pgm_filename="test_maze.pgm",
        yaml_filename="test_maze.yaml"
    )
    await test_map.insert()
    print(f"Mapa creado: {test_map.name} (ID: {test_map.id})")

    # Crear Puntos de Prueba para ese mapa
    points_to_create = [
        Point(name="Punto de Inicio", map_id=test_map.id, x=1.0, y=1.0, yaw=0.5),
        Point(name="Zona de Carga 1", map_id=test_map.id, x=5.3, y=-2.1, yaw=1.57),
        Point(name="Esquina de Navegación", map_id=test_map.id, x=-3.0, y=4.5, yaw=-0.78),
    ]

    await Point.insert_many(points_to_create)
    print(f"Creados {len(points_to_create)} puntos para el mapa '{test_map.name}'.")

    print("\nBase de datos poblada con éxito.")

if __name__ == "__main__":
    asyncio.run(seed())