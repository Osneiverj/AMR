import { createContext, useContext, useState, useEffect } from "react";
import { MapsAPI, PointsAPI, MissionsAPI } from "../services/api";
import { useAuth } from "../AuthContext";

const Ctx = createContext(null);
export const useData = () => useContext(Ctx);

export function DataProvider({ children }) {
  const [maps, setMaps] = useState([]);
  const [selectedMap, setSelectedMap] = useState(null);
  const [points, setPoints] = useState([]);
  const [missions, setMissions] = useState([]);
  const { token } = useAuth();

  useEffect(() => {
    if (token) {
      MapsAPI.list(token).then(setMaps).catch(() => setMaps([]));
    }
  }, [token]);

  useEffect(() => {
    if (!selectedMap || !token) return;
    PointsAPI.list(selectedMap, token).then(setPoints);
    MissionsAPI.list(selectedMap, token).then(setMissions);
  }, [selectedMap, token]);

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
