import { useState } from 'react';
import ros from '../services/rosService';

export default function NewMapButton() {
  const [busy, setBusy] = useState(false);

    // Iniciar borrando el mapa y activando modo interactivo
    const startMapping = async () => {
    setBusy(true);
    try {
        // 1. Clear (descarta mapas previos)
        await ros.callService('/slam_toolbox/clear_changes', 'slam_toolbox/srv/Clear');
        // 2. (Opcional) toggle_interactive para reiniciar pose graph
        await ros.callService('/slam_toolbox/toggle_interactive_mode', 'slam_toolbox/srv/ToggleInteractive');
        alert('SLAM listo: conduce el robot para mapear.');
    } catch (e) {
        alert('Error al controlar SLAM:\n' + e);
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
