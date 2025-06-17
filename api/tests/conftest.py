# api/tests/conftest.py
import pytest
import pytest_asyncio # pytest-asyncio ahora usa pytest_asyncio.fixture
from httpx import AsyncClient
from typing import AsyncGenerator, Generator # Para tipado de fixtures

# Importa tu aplicación FastAPI y la configuración/inicialización de la BD
# Asegúrate de que el PYTHONPATH esté configurado para que pytest encuentre 'app'
# o ajusta los imports si ejecutas pytest desde un directorio diferente.
from app.main import app 
from app.core.config import settings 
from app.core.db import init_db
import asyncio
from motor.motor_asyncio import AsyncIOMotorClient

# Define la URI para la base de datos de prueba.
# Es crucial que sea DIFERENTE de tu base de datos de desarrollo/producción.
TEST_MONGO_URI = settings.mongo_uri + "_test" 

@pytest_asyncio.fixture(scope="session", autouse=True)
async def initialize_test_database() -> AsyncGenerator[None, None]:
    """
    Fixture de sesión para inicializar y limpiar la base de datos de prueba.
    Se ejecuta una vez por sesión de pruebas.
    'autouse=True' asegura que se ejecute automáticamente.
    """
    # Guarda la URI original y la sobrescribe temporalmente para las pruebas
    original_mongo_uri = settings.mongo_uri
    settings.mongo_uri = TEST_MONGO_URI

    # Inicializa el cliente con la URI de prueba
    test_client = AsyncIOMotorClient(settings.mongo_uri)
    # Obtiene el nombre de la base de datos (ej. 'agv_test')
    db_name_to_test = test_client.get_default_database().name
    
    # Llama a init_db (que ahora usará TEST_MONGO_URI a través de settings.mongo_uri)
    # para configurar Beanie con los modelos en la base de datos de prueba.
    await init_db() 
    print(f"INFO: Base de datos de prueba '{db_name_to_test}' en '{settings.mongo_uri}' inicializada para la sesión de pruebas.")

    yield # Las pruebas se ejecutan aquí

    # Limpieza: Borra la base de datos de prueba después de que todas las pruebas de la sesión hayan terminado.
    print(f"INFO: Limpiando la base de datos de prueba '{db_name_to_test}'...")
    await test_client.drop_database(db_name_to_test)
    await test_client.close() # Cierra la conexión del cliente de prueba
    print(f"INFO: Base de datos de prueba '{db_name_to_test}' limpiada y conexión cerrada.")
    
    # Restaura la URI original en los settings
    settings.mongo_uri = original_mongo_uri


@pytest_asyncio.fixture(scope="session")
def event_loop() -> asyncio.AbstractEventLoop:
    """Create an event loop for the entire test session."""
    loop = asyncio.get_event_loop_policy().new_event_loop()
    yield loop
    loop.close()


@pytest_asyncio.fixture(scope="function") 
async def async_client() -> AsyncGenerator[AsyncClient, None]:
    """
    Fixture de función para proporcionar un cliente HTTP asíncrono (httpx.AsyncClient)
    configurado para realizar solicitudes a la aplicación FastAPI en memoria.
    Se ejecuta para cada función de prueba, asegurando un cliente limpio.
    """
    async with AsyncClient(app=app, base_url="http://testserver") as client:
        # "http://testserver" es una URL base ficticia que httpx utiliza internamente.
        # Las solicitudes no salen a la red real cuando 'app' es proporcionado.
        yield client
        # Aquí se podría añadir limpieza a nivel de función si fuera necesario,
        # pero la limpieza de la BD a nivel de sesión es generalmente suficiente
        # para el estado de la base de datos entre pruebas de endpoint.