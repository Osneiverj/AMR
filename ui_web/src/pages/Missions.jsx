import { useState } from "react";
import { useData } from "../context/DataContext";
import { MissionsAPI } from "../services/api";

export default function Missions() {
  const { selectedMap, points, missions, setMissions } = useData();
  const [form, setForm] = useState({ name: "", loop: false, sequence: [] });

  if (!selectedMap) return <p className="p-4">Selecciona un mapa.</p>;

  const toggleSeq = id =>
    setForm(f => ({
      ...f,
      sequence: f.sequence.includes(id)
        ? f.sequence.filter(x => x !== id)
        : [...f.sequence, id]
    }));

  const save = async e => {
    e.preventDefault();
    await MissionsAPI.create({ ...form, map_id: selectedMap });
    setMissions(await MissionsAPI.list(selectedMap));
    setForm({ name: "", loop: false, sequence: [] });
  };

  return (
    <div className="p-4 space-y-4">
      <h2 className="text-xl font-semibold">Misiones</h2>
      <form onSubmit={save} className="space-y-2">
        <input
          value={form.name}
          onChange={e => setForm({ ...form, name: e.target.value })}
          placeholder="Nombre"
          required
          className="input"
        />
        <label className="flex items-center gap-2">
          <input
            type="checkbox"
            checked={form.loop}
            onChange={e => setForm({ ...form, loop: e.target.checked })}
          />
          loop
        </label>
        <fieldset className="border p-2">
          <legend>Seleccionar puntos</legend>
          {points.map(p => (
            <label key={p._id} className="block">
              <input
                type="checkbox"
                checked={form.sequence.includes(p._id)}
                onChange={() => toggleSeq(p._id)}
              />
              {` ${p.name}`}
            </label>
          ))}
        </fieldset>
        <button className="btn">Guardar misión</button>
      </form>
      <ul className="space-y-1">
        {missions.map(m => (
          <li key={m._id} className="flex justify-between">
            <span>{m.name}</span>
            <button
              onClick={async () => {
                await MissionsAPI.delete(m._id);
                setMissions(await MissionsAPI.list(selectedMap));
              }}
              className="text-red-600"
            >
              🗑
            </button>
          </li>
        ))}
      </ul>
    </div>
  );
}
