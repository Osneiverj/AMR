import { useState } from "react";
import { useData } from "../context/DataContext";
import { PointsAPI } from "../services/api";

export default function Points() {
  const { selectedMap, points, setPoints } = useData();
  const emptyPose = { x: 0, y: 0, z: 0, q1: 0, q2: 0, q3: 0, q4: 1 };
  const [form, setForm] = useState({ name: "", type: "way", target: emptyPose });

  if (!selectedMap) return <p className="p-4">Selecciona un mapa.</p>;

  const onChange = e => setForm({ ...form, [e.target.name]: e.target.value });
  const onPoseChange = e =>
    setForm({ ...form, target: { ...form.target, [e.target.name]: parseFloat(e.target.value) } });

  const save = async e => {
    e.preventDefault();
    await PointsAPI.create({ ...form, map_id: selectedMap });
    setPoints(await PointsAPI.list(selectedMap));
    setForm({ name: "", type: "way", target: emptyPose });
  };

  return (
    <div className="p-4 space-y-4">
      <h2 className="text-xl font-semibold">Puntos</h2>
      <form onSubmit={save} className="grid gap-2 grid-cols-4">
        <input
          name="name"
          value={form.name}
          onChange={onChange}
          placeholder="Nombre"
          required
          className="input col-span-2"
        />
        <select name="type" value={form.type} onChange={onChange} className="input">
          <option value="way">way</option>
          <option value="dock">dock</option>
          <option value="station">station</option>
        </select>
        {[
          "x",
          "y",
          "z",
          "q1",
          "q2",
          "q3",
          "q4"
        ].map(k => (
          <input
            key={k}
            name={k}
            value={form.target[k]}
            onChange={onPoseChange}
            step="0.01"
            className="input"
            placeholder={k}
          />
        ))}
        <button className="btn col-span-4">Guardar</button>
      </form>
      <ul className="space-y-1">
        {points.map(p => (
          <li key={p._id} className="flex justify-between">
            <span>{p.name}</span>
            <button
              onClick={async () => {
                await PointsAPI.delete(p._id);
                setPoints(await PointsAPI.list(selectedMap));
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
