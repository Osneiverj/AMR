import Map from '../components/Map';
import TelemetryCard from '../components/TelemetryCard';
import LogTable from '../components/LogTable';
import Jog from '../components/Jog';
import NewMapButton from '../components/NewMapButton';
import SaveMapButton from '../components/SaveMapButton';

export default function Dashboard() {
  return (
    <div className="grid gap-4 lg:grid-cols-4">
      {/* ----- Columna principal (mapa) ----- */}
      <div className="lg:col-span-3 space-y-4">
        <Map />
        <div className="flex gap-2">
          <TelemetryCard />
          <NewMapButton />
          <SaveMapButton />         {/* ← NUEVO */}
        </div>
        <LogTable />
      </div>

      {/* ----- Columna derecha (jog) ----- */}
      <div className="lg:col-span-1">
        <Jog />
      </div>
    </div>
  );
}
