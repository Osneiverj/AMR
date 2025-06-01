# api/tests/unit/domain/test_maps_service.py
import pytest
from unittest.mock import AsyncMock, patch # AsyncMock para funciones async
from datetime import datetime

# Importa la clase y la función a probar del servicio de mapas
from app.domain.maps.model import Map
from app.domain.maps.service import save as save_map_service 
# No necesitamos settings aquí porque estamos mockeando la interacción con la BD y el filesystem

@pytest.mark.asyncio
async def test_save_map_service_creates_and_inserts_document(mocker):
    """
    Prueba la lógica de la función de servicio 'save' para mapas,
    mockeando la llamada de inserción a la base de datos.
    """
    original_map_name = "unit_test_map_service_01"
    pgm_file_name_on_disk = "sanitized_or_direct_pgm_name.pgm"
    yaml_file_name_on_disk = "sanitized_or_direct_yaml_name.yaml"

    # Preparamos el mock para el método 'insert' de la instancia del documento Map.
    # Cuando 'await doc.insert()' es llamado en el servicio, nuestro mock se ejecutará.
    # Usamos AsyncMock porque 'insert' es un método async.
    
    # Guardamos una referencia al método insert original si quisiéramos
    # original_insert = Map.insert 
    # No es necesario aquí ya que lo reemplazamos completamente para esta prueba.

    # Mockeamos el método 'insert' de la clase Documento (Map hereda de Document)
    # Haremos que el mock asigne un ID simulado al documento cuando se llame.
    async def mock_successful_insert(document_instance):
        # Simula que Beanie/MongoDB asigna un ID al documento al insertarlo.
        # El tipo de ID depende de cómo lo definas en tu modelo Beanie (PydanticObjectId o str).
        # Asumamos que es un str para simplificar el mock.
        document_instance.id = "mock_db_id_12345"
        # El campo 'created' debería ser manejado por el default_factory en el modelo Map,
        # por lo que se establecerá cuando se cree la instancia de 'Map' en el servicio.
        return document_instance # Beanie insert suele devolver la instancia

    # Usamos patch.object para mockear el método 'insert' en todas las instancias de Map
    # durante esta prueba.
    # autospec=True ayuda a asegurar que el mock tenga la misma firma que el original.
    mocker.patch.object(Map, "insert", new_callable=AsyncMock, side_effect=mock_successful_insert, autospec=True)

    # Llamamos a la función del servicio que estamos probando
    saved_map_document = await save_map_service(
        original_name=original_map_name,
        pgm_filename=pgm_file_name_on_disk,
        yaml_filename=yaml_file_name_on_disk
    )

    # Verificaciones sobre el documento devuelto por el servicio
    assert saved_map_document is not None
    assert saved_map_document.name == original_map_name
    assert saved_map_document.pgm == pgm_file_name_on_disk
    assert saved_map_document.yaml == yaml_file_name_on_disk
    
    # Verifica que el ID fue asignado por nuestro mock (a través del side_effect)
    assert saved_map_document.id == "mock_db_id_12345" 
    
    # Verifica que el campo 'created' fue establecido (por default_factory) y es una datetime
    assert isinstance(saved_map_document.created, datetime)

    # Verifica que el método 'insert' mockeado fue llamado una vez
    Map.insert.assert_called_once()
    
    # Verifica que se llamó con la instancia correcta (el primer argumento de insert es 'self')
    # El argumento `ANY` de unittest.mock no está disponible directamente en pytest sin más setup,
    # pero podemos inspeccionar la llamada si es necesario.
    # called_with_instance = Map.insert.call_args[0][0]
    # assert called_with_instance.name == original_map_name