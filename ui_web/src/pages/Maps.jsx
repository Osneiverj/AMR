import { useState, useEffect } from "react";
import { useData } from "../context/DataContext";
import { MapsAPI } from "../services/api";
import { useAuth } from "../AuthContext";

export default function Maps() {
  const { setSelectedMap } = useData();
  const { token, user } = useAuth();

  const [availableMaps, setAvailableMaps] = useState([]);
  const [selectedLocalMap, setSelectedLocalMap] = useState("");
  const [isLoading, setIsLoading] = useState(false);
  const [message, setMessage] = useState("");

  useEffect(() => {
    if (token) {
      MapsAPI.listAvailable(token)
        .then(maps => {
          setAvailableMaps(maps);
          if (maps.length > 0) setSelectedLocalMap(maps[0]);
        })
        .catch(err => setMessage(`Error al cargar mapas: ${err.message}`));
    }
  }, [token]);

  const handleActivateMap = async () => {
    if (!selectedLocalMap) {
      setMessage("Por favor, selecciona un mapa.");
      return;
    }
    setIsLoading(true);
    setMessage(`Activando el mapa '${selectedLocalMap}'...`);
    try {
      const res = await MapsAPI.activate(selectedLocalMap, token);
      setMessage(res.message || "Mapa activado con éxito.");
      // opcional: establece en contexto el mapa seleccionado
      setSelectedMap(null); // limpiar selección previa
    } catch (err) {
      setMessage(`Error al activar el mapa: ${err.message}`);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className="p-4 space-y-4">
      <h2 className="text-xl font-semibold">Gestión de Mapas</h2>
      <div className="bg-white p-4 rounded-lg shadow">
        <h3 className="font-medium mb-2">Activar Mapa en ROS</h3>
        <div className="flex gap-2 items-center">
          <select
            value={selectedLocalMap}
            onChange={e => setSelectedLocalMap(e.target.value)}
            className="input flex-grow"
            disabled={availableMaps.length === 0}
          >
            {availableMaps.length === 0 ? (
              <option>No hay mapas disponibles</option>
            ) : (
              availableMaps.map(m => (
                <option key={m} value={m}>
                  {m}
                </option>
              ))
            )}
          </select>
          <button
            onClick={handleActivateMap}
            disabled={isLoading || availableMaps.length === 0}
            className="btn-primary"
          >
            {isLoading ? "Activando..." : "🚀 Activar Mapa"}
          </button>
        </div>
      </div>
      {message && <p className="text-gray-600 bg-gray-100 p-2 rounded">{message}</p>}
    </div>
  );
}
