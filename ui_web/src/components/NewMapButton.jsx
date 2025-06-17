import { useState } from 'react';
import ros from '../services/rosService';

export default function NewMapButton() {
  const [busy, setBusy] = useState(false);

  const startMapping = async () => {
    setBusy(true);
    try {
      await ros.callService('/ui/start_mapping_mode', 'std_srvs/srv/Empty');
      alert('Modo mapeo iniciado: conduce el robot para crear el mapa.');
    } catch (e) {
      alert('Error al iniciar mapeo:\n' + e);
    } finally {
      setBusy(false);
    }
  };


  return (
    <button
      onClick={startMapping}
      disabled={busy}
      className="bg-blue-600 hover:bg-blue-700 text-white px-4 py-2 rounded-lg shadow disabled:opacity-50">
      {busy ? 'Iniciando…' : '🗺️  Crear nuevo mapa'}
    </button>
  );
}
