const BASE = import.meta.env.VITE_API_URL ?? "http://localhost:5000";

async function req(method, url, body, isForm = false, token) {
  const init = {
    method,
    headers: token ? { Authorization: `Bearer ${token}` } : {}
  };
  if (body) {
    if (isForm) {
      init.body = body;
    } else {
      init.body = JSON.stringify(body);
      init.headers["Content-Type"] = "application/json";
    }
  }
  const res = await fetch(BASE + url, init);
  if (!res.ok) throw new Error(await res.text());
  return res.status === 204 ? null : res.json();
}

export const MapsAPI = {
  list: token => req("GET", "/maps", null, false, token),
  upload: (name, pgm, yaml, token) => {
    const f = new FormData();
    f.append("name", name);
    f.append("pgm", pgm);
    f.append("yaml", yaml);
    return req("POST", "/maps", f, true, token);
  },
  delete: (name, token) => req("DELETE", `/maps/${name}`, null, false, token),
  listAvailable: token => req("GET", "/maps/available", null, false, token),
  activate: (mapName, token) => req("POST", `/maps/${mapName}/activate`, null, false, token)
};

export const PointsAPI = {
  list: (mapId, token) => req("GET", `/points${mapId ? `?map_id=${mapId}` : ""}`, null, false, token),
  create: (p, token) => req("POST", "/points", p, false, token),
  delete: (id, token) => req("DELETE", `/points/${id}`, null, false, token)
};

export const MissionsAPI = {
  list: (mapId, token) => req("GET", `/missions${mapId ? `?map_id=${mapId}` : ""}`, null, false, token),
  create: (m, token) => req("POST", "/missions", m, false, token),
  delete: (id, token) => req("DELETE", `/missions/${id}`, null, false, token)
};
