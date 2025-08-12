# api/app/domain/missions/service.py
import json
import os
from beanie import PydanticObjectId
from typing import List
from jsonschema import validate, ValidationError
from .model import Mission
from app.domain.maps.model import Map # Necesario para la validación

# Cargar el esquema de misión una sola vez al iniciar el módulo
SCHEMA_PATH = os.path.join(os.path.dirname(__file__), 'mission_schema.json')
with open(SCHEMA_PATH, 'r') as f:
    MISSION_SCHEMA = json.load(f)

# ------------------------------------------------------------------
# CREATE
# ------------------------------------------------------------------
async def create(data: dict) -> Mission:
    # 1. Validar que el mapa existe
    if not await Map.get(data["map_id"]):
        raise ValueError(f"Mapa inexistente con ID: {data['map_id']}")

    # 2. Validar la definición de la misión contra el JSON schema
    try:
        validate(instance=data.get("definition"), schema=MISSION_SCHEMA)
    except ValidationError as e:
        raise ValueError(f"Error de validación en la definición de la misión: {e.message}")

    # 3. Crear documento (la lógica de validación de puntos se elimina)
    doc = Mission(**data)
    await doc.insert()
    return doc

from app.core.ros import get_ros_client
import roslibpy
import json

async def run_mission(mission_id: PydanticObjectId) -> dict:
    mission = await Mission.get(mission_id)
    if not mission:
        raise ValueError("Mission not found")

    ros_client = get_ros_client()
    if not ros_client.is_connected:
        raise ConnectionError("No se pudo conectar con el orquestador de ROS.")

    action_client = roslibpy.ActionClient(ros_client,
                                          '/execute_mission',
                                          'orchestrator/ExecuteMission')

    goal = roslibpy.Goal(action_client, roslibpy.Message({
        'mission_json': json.dumps(mission.definition)
    }))

    # No bloquearemos la API esperando el resultado, solo enviamos el objetivo.
    # El frontend se encargará de escuchar el feedback.
    goal.send()

    return {"status": "sent_to_ros", "mission_id": str(mission.id), "goal_id": goal.goal_id}


# ------------------------------------------------------------------
# READ (list)
# ------------------------------------------------------------------
async def list_all(map_id: str | None = None) -> List[Mission]: # Pydantic v1 compatible
    if map_id:
        return await Mission.find(Mission.map_id == map_id).to_list()
    return await Mission.find_all().to_list()


# ------------------------------------------------------------------
# DELETE
# ------------------------------------------------------------------
async def remove(mission_id: PydanticObjectId) -> bool: # Devuelve booleano
    doc = await Mission.get(mission_id)
    if not doc:
        return False
    await doc.delete()
    return True