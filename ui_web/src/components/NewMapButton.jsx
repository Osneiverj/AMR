import { useState } from 'react';
import ros from '../services/rosService';

export default function NewMapButton() {
  const [busy, setBusy] = useState(false);

  // Solicita al Orchestrator activar el modo mapeo
  const startMapping = async () => {
    setBusy(true);
    try {
      await ros.callService('/ui/start_slam', 'std_srvs/srv/Empty');
      alert('SLAM listo: conduce el robot para mapear.');
    } catch (e) {
      alert('Error al iniciar SLAM:\n' + e);
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
