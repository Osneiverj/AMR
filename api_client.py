import asyncio
import httpx

API_BASE_URL = "http://localhost:5000"
MAP_NAME_TO_CHECK = "mapa_de_prueba_api"

async def main():
    async with httpx.AsyncClient(base_url=API_BASE_URL) as client:
        # 1. Autenticación
        print("Autenticando...")
        try:
            auth_response = await client.post("/auth/login", data={"username": "admin", "password": "admin"})
            auth_response.raise_for_status()
            token = auth_response.json()["access_token"]
            headers = {"Authorization": f"Bearer {token}"}
            print("Autenticación exitosa.")
        except Exception as e:
            print(f"Error de autenticación: {e}")
            return

        # 2. Buscar el mapa
        print(f"Buscando el mapa '{MAP_NAME_TO_CHECK}'...")
        map_id = None
        try:
            get_maps_response = await client.get("/maps/", headers=headers)
            get_maps_response.raise_for_status()
            maps = get_maps_response.json()
            existing_map = next((m for m in maps if m["name"] == MAP_NAME_TO_CHECK), None)

            if existing_map:
                map_id = existing_map.get("id") or existing_map.get("_id")
                print(f"Mapa encontrado. ID: {map_id}")
            else:
                print(f"El mapa '{MAP_NAME_TO_CHECK}' no fue encontrado.")
                return
        except Exception as e:
            print(f"Error al buscar el mapa: {e}")
            return

        # 3. Consultar los puntos para ese mapa
        if map_id:
            print(f"\nConsultando puntos para el mapa ID: {map_id}...")
            try:
                get_points_response = await client.get(f"/points/?map_id={map_id}", headers=headers)
                get_points_response.raise_for_status()
                points = get_points_response.json()
                print(f"Consulta exitosa. Se encontraron {len(points)} puntos:")
                for point in points:
                    print(f"  - ID: {point.get('_id')}, Nombre: {point.get('name')}")
            except Exception as e:
                print(f"Error al consultar los puntos: {e}")

if __name__ == "__main__":
    asyncio.run(main())
