import { useState } from 'react';
import ros from '../services/rosService';

export default function SaveMapButton() {
  const [busy, setBusy] = useState(false);

  const saveMap = async () => {
    const name = prompt('Nombre para el mapa:');   // null si Cancel
    if (!name) return;

    setBusy(true);
    try {
      const res = await ros.callService(
        '/slam_toolbox/save_map',
        'slam_toolbox/srv/SaveMap',
        { name: { data: name } }                  // guarda name.pgm/yaml
      );
      switch (res.result) {
        case 0:
          alert(`Mapa guardado como "${name}".`);
          break;
        case 1:
          alert('No se recibió ningún mapa: mueve el robot primero.');
          break;
        default:
          alert('Fallo desconocido al guardar el mapa.');
      }
    } catch (e) {
      alert('Servicio no disponible:\n' + e);
    } finally {
      setBusy(false);
    }
  };

  return (
    <button
      onClick={saveMap}
      disabled={busy}
      className="bg-green-600 hover:bg-green-700 text-white px-4 py-2 rounded-lg shadow disabled:opacity-50">
      {busy ? 'Guardando…' : '💾  Guardar mapa'}
    </button>
  );
}
