import { useRef } from "react";
import { useData } from "../context/DataContext";
import { MapsAPI } from "../services/api";

export default function Maps() {
  const { maps, setMaps, setSelectedMap } = useData();
  const nameRef = useRef();
  const pgmRef = useRef();
  const yamlRef = useRef();

  const upload = async e => {
    e.preventDefault();
    await MapsAPI.upload(
      nameRef.current.value,
      pgmRef.current.files[0],
      yamlRef.current.files[0]
    );
    setMaps(await MapsAPI.list());
    e.target.reset();
  };

  return (
    <div className="p-4 space-y-4">
      <h2 className="text-xl font-semibold">Mapas</h2>
      <form onSubmit={upload} className="flex gap-2 flex-wrap">
        <input ref={nameRef} placeholder="Nombre" required className="input" />
        <input ref={pgmRef} type="file" accept=".pgm" required />
        <input ref={yamlRef} type="file" accept=".yaml" required />
        <button className="btn">Subir</button>
      </form>
      <ul className="space-y-1">
        {maps.map(m => (
          <li key={m._id} className="flex justify-between">
            <span
              onClick={() => setSelectedMap(m._id)}
              className="cursor-pointer hover:underline"
            >
              {m.name}
            </span>
            <button
              onClick={async () => {
                await MapsAPI.delete(m.name);
                setMaps(await MapsAPI.list());
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
