# api/tests/integration/test_maps_router.py
import pytest
from httpx import AsyncClient
from fastapi import status # Para usar códigos de estado HTTP estándar
# Si necesitas verificar los modelos Beanie en la BD directamente en una prueba (menos común para pruebas de router):
# from app.domain.maps.model import Map 

@pytest.mark.asyncio
async def test_list_maps_empty_initially(async_client: AsyncClient):
    """
    Prueba que la lista de mapas esté vacía inicialmente (asumiendo BD de prueba limpia).
    """
    response = await async_client.get("/maps/")
    assert response.status_code == status.HTTP_200_OK
    assert response.json() == [] # Espera una lista vacía

@pytest.mark.asyncio
async def test_upload_map_success(async_client: AsyncClient):
    """
    Prueba la subida exitosa de un nuevo mapa.
    """
    map_name = "integration_test_map_01"
    # Contenido simulado para los archivos de mapa
    dummy_pgm_content = b"#P5\n1 1\n255\n\0" # Un PGM binario mínimo y válido (1x1 pixel)
    dummy_yaml_content = f"image: {map_name}.pgm\nresolution: 0.05\norigin: [-1.0, -1.0, 0.0]"

    files_to_upload = {
        "pgm": (f"{map_name}.pgm", dummy_pgm_content, "image/x-portable-graymap"), # Content-type correcto para PGM
        "yaml": (f"{map_name}.yaml", dummy_yaml_content.encode('utf-8'), "application/x-yaml"),
    }
    form_data = {"name": map_name}

    response = await async_client.post("/maps/", files=files_to_upload, data=form_data)

    assert response.status_code == status.HTTP_201_CREATED, f"Error: {response.text}"
    response_data = response.json()
    
    assert response_data["name"] == map_name
    # Asumiendo que el servicio guarda los nombres de archivo tal como se reciben
    # o según la lógica de sanitización que implementaste en el router.
    # Si sanitizaste, el nombre del archivo aquí podría ser diferente de map_name.
    assert response_data["pgm"].endswith(".pgm") 
    assert response_data["yaml"].endswith(".yaml")
    assert "id" in response_data
    assert "created" in response_data # Verifica que el campo 'created' (Paso 5) esté presente

    # Opcional: Verifica que el mapa ahora aparece en la lista
    list_response = await async_client.get("/maps/")
    assert list_response.status_code == status.HTTP_200_OK
    maps_list = list_response.json()
    assert len(maps_list) == 1
    assert maps_list[0]["name"] == map_name

@pytest.mark.asyncio
async def test_upload_map_duplicate_name_fails(async_client: AsyncClient):
    """
    Prueba que no se puede subir un mapa con un nombre que ya existe.
    """
    map_name = "duplicate_map_test_01"
    dummy_pgm_content = b"#P5\n1 1\n255\n\0"
    dummy_yaml_content = f"image: {map_name}.pgm\nresolution: 0.05\norigin: [-1.0, -1.0, 0.0]"
    files = {
        "pgm": (f"{map_name}.pgm", dummy_pgm_content, "image/x-portable-graymap"),
        "yaml": (f"{map_name}.yaml", dummy_yaml_content.encode('utf-8'), "application/x-yaml"),
    }
    data = {"name": map_name}

    # Primera subida (debería ser exitosa)
    response1 = await async_client.post("/maps/", files=files, data=data)
    assert response1.status_code == status.HTTP_201_CREATED, f"Primera subida falló: {response1.text}"

    # Segunda subida con el mismo nombre (debería fallar con 409 Conflict)
    response2 = await async_client.post("/maps/", files=files, data=data)
    assert response2.status_code == status.HTTP_409_CONFLICT
    response_detail = response2.json().get("detail", "")
    assert "Nombre de mapa existente" in response_detail or "existente" in response_detail.lower()