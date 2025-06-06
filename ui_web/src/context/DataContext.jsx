import { createContext, useContext, useState, useEffect } from "react";
import { MapsAPI, PointsAPI, MissionsAPI } from "../services/api";

const Ctx = createContext(null);
export const useData = () => useContext(Ctx);

export function DataProvider({ children }) {
  const [maps, setMaps] = useState([]);
  const [selectedMap, setSelectedMap] = useState(null);
  const [points, setPoints] = useState([]);
  const [missions, setMissions] = useState([]);

  useEffect(() => {
    MapsAPI.list().then(setMaps);
  }, []);

  useEffect(() => {
    if (!selectedMap) return;
    PointsAPI.list(selectedMap).then(setPoints);
    MissionsAPI.list(selectedMap).then(setMissions);
  }, [selectedMap]);

  return (
    <Ctx.Provider
      value={{
        maps,
        setMaps,
        selectedMap,
        setSelectedMap,
        points,
        setPoints,
        missions,
        setMissions
      }}
    >
      {children}
    </Ctx.Provider>
  );
}
